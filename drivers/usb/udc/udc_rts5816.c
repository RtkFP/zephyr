/*
 * Copyright (c) 2024 Realtek Semiconductor, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/usb/usb_device.h>
#include "udc_common.h"
#include "udc_rts5816_lib.h"

#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/cache.h>
LOG_MODULE_REGISTER(udc_rts5816, CONFIG_UDC_DRIVER_LOG_LEVEL);

#define DT_DRV_COMPAT realtek_rts5816_usbd

struct rts_drv_event {
	enum event_type evt_type;
	uint8_t ep;
};

K_MSGQ_DEFINE(drv_msgq, sizeof(struct rts_drv_event), CONFIG_UDC_RTS_MAX_QMESSAGES, sizeof(void *));

/*
 * Structure for holding controller configuration items that can remain in
 * non-volatile memory. This is usually accessed as
 *   const struct udc_rts_config *config = dev->config;
 */

struct udc_rts_config {
	size_t num_of_eps;
	struct udc_ep_config *ep_cfg_in;
	struct udc_ep_config *ep_cfg_out;
	void (*make_thread)(const struct device *dev);
	int speed_idx;
	void (*irq_enable_func)(const struct device *dev);
	void (*irq_disable_func)(const struct device *dev);
	USB_TypeDef usb;
};

struct udc_rts_bulkout_param {
	uint16_t recv_len;
	bool last_sp;
};

/*
 * Structure to hold driver private data.
 * Note that this is not accessible via dev->data, but as
 *   struct udc_rts_data *priv = udc_get_private(dev);
 */
struct udc_rts_data {
	struct k_thread thread_data;
	struct usb_setup_packet setup_pkt;
	struct udc_rts_bulkout_param bulkout[2];
};

static const uint8_t ep_lnum_table[] = {EP0, EPA, EPB, EPC, EPD, EPE, EPF, EPG};
static const uint8_t ep_dir_table[] = {0,           USB_EPA_DIR, USB_EPB_DIR, USB_EPC_DIR,
				       USB_EPD_DIR, USB_EPE_DIR, USB_EPF_DIR, USB_EPG_DIR};

#define sys_write32_mask(val, mask, reg)                                                           \
	({                                                                                         \
		uint32_t v = sys_read32(reg) & ~(mask);                                            \
		sys_write32(v | ((val) & (mask)), reg);                                            \
	})

static uint8_t rts_ep_idx(uint8_t ep)
{
	uint8_t ep_idx = USB_EP_GET_IDX(ep);
	uint8_t ep_dir = USB_EP_GET_DIR(ep);
	uint8_t idx_num = 0;

	if (ep_idx == EP0) {
		return 0;
	}

	for (uint8_t i = 1; i < MAX_NUM_EP; i++) {
		if ((ep_idx == ep_lnum_table[i]) && (ep_dir == ep_dir_table[i])) {
			idx_num = i;
			break;
		}
	}
	return idx_num;
}

static void rts_ep_en(const struct device *dev, uint8_t ep_idx, struct udc_ep_config *const ep_cfg)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;
	struct udc_rts_data *const priv = udc_get_private(dev);
	enum usb_dc_ep_transfer_type type = ep_cfg->attributes & USB_EP_TRANSFER_TYPE_MASK;
	uint16_t mps = ep_cfg->mps;

	if (type == USB_EP_TYPE_CONTROL) {
		rts_ep_rst(usb, ep_idx);
		rts_ep_cfg_mps(usb, EP0, USB_EP0_MPS);
		rts_ep_nakout(usb, ep_idx);
	} else if (type == USB_EP_TYPE_BULK) {
		rts_ep_fifo_flush(usb, ep_idx);
		rts_ep_fifo_en(usb, ep_idx);
		rts_ep_cfg_lnum(usb, ep_idx, ep_lnum_table[ep_idx]);
		rts_ep_cfg_mps(usb, ep_idx, mps);
		rts_ep_clr_int_sts(usb, ep_idx, 0xff);
		rts_ep_force_toggle(usb, ep_idx, USB_EP_DATA_ID_DATA0);
		if (ep_idx == EPA) {
			rts_ep_nakout(usb, ep_idx);
			memset((uint8_t *)&priv->bulkout[0], 0,
			       sizeof(struct udc_rts_bulkout_param));
		} else if (ep_idx == EPE) {
			rts_ep_nakout(usb, ep_idx);
			memset((uint8_t *)&priv->bulkout[1], 0,
			       sizeof(struct udc_rts_bulkout_param));
		}
		rts_ep_stall(usb, ep_idx, false);
		rts_ep_cfg_en(usb, ep_idx, true);
		rts_ep_rst(usb, ep_idx);
	} else if (type == USB_EP_TYPE_INTERRUPT) {
		rts_ep_cfg_lnum(usb, ep_idx, ep_lnum_table[ep_idx]);
		if (ep_idx == EPG) {
			rts_ep_cfg_mps(usb, ep_idx, USB_INT_EPG_MPS);
		} else {
			rts_ep_cfg_mps(usb, ep_idx, mps);
		}
		rts_ep_clr_int_sts(usb, ep_idx, 0xff);
		rts_ep_force_toggle(usb, ep_idx, USB_EP_DATA_ID_DATA0);
		rts_ep_stall(usb, ep_idx, false);
		rts_ep_cfg_en(usb, ep_idx, true);
		rts_ep_rst(usb, ep_idx);
	} else {
		return;
	}
}

static void rts_usb_init(const struct device *dev, bool reset_from_pwron)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;

	rts_usb_disconnect(usb);
	rts_usb_phy_init(usb);
	rts_ls_clr_int_sts(usb, 0xFFFFFFFFUL);
	rts_ep_clr_int_sts(usb, EP0, 0xFFFFFFFFUL);
	rts_ls_set_int_en(usb, IE_SUSPND | IE_SE0RST | IE_RESUME, true);
	rts_ep_set_int_en(usb, EP0, USB_EP0_SETUP_PACKET_INT, true);
}

static void rts_usb_handle_rst(const struct device *dev)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;
	struct udc_data *data = dev->data;

	rts_ls_clr_int_sts(usb, USB_LS_PORT_RST_INT);
	udc_submit_event(dev, UDC_EVT_RESET, 0);

	data->caps.addr_before_status = true;
}

static int rts_ep0_start_tx(const struct device *dev)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;
	struct net_buf *buf;
	uint8_t trans_len;

	buf = udc_buf_peek(dev, USB_CONTROL_EP_IN);

	if (buf->len) {
		trans_len = MIN(cfg->ep_cfg_in[0].mps, buf->len);

	} else {
		trans_len = 0;
		if (udc_ep_buf_has_zlp(buf)) {
			udc_ep_buf_clear_zlp(buf);
		} else {
			return -ENOBUFS;
		}
	}

	rts_ep0_start_tx_sub(usb, buf->data, trans_len);
	net_buf_pull(buf, trans_len);

	return trans_len;
}

static void rts_usb_handle_suspend(const struct device *dev)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;
	/* TODO */
	rts_ls_clr_int_sts(usb, USB_LS_SUSPEND_INT);
	udc_set_suspended(dev, true);
	udc_submit_event(dev, UDC_EVT_SUSPEND, 0);
}

static void rts_usb_handle_resume(const struct device *dev)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;

	/* TODO */
	rts_ls_clr_int_sts(usb, USB_LS_RESUME_INT);
	udc_set_suspended(dev, false);
	udc_submit_event(dev, UDC_EVT_RESUME, 0);
}

static void rts_usb_handle_sof(const struct device *dev)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;

	rts_ls_clr_int_sts(usb, USB_LS_SOF_INT);
	udc_submit_event(dev, UDC_EVT_SOF, 0);
}

/* put buf->data at tail of ctrl ep out fifo(standard) */
static int rts_ctrl_feed_dout(const struct device *dev, const size_t length)
{
	struct udc_ep_config *ep_cfg = udc_get_ep_cfg(dev, USB_CONTROL_EP_OUT);
	struct net_buf *buf;

	buf = udc_ctrl_alloc(dev, USB_CONTROL_EP_OUT, length);
	if (buf == NULL) {
		return -ENOMEM;
	}

	udc_buf_put(ep_cfg, buf);

	return 0;
}

/* set up event */
static int rts_handle_evt_setup(const struct device *dev)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;
	struct udc_rts_data *const priv = udc_get_private(dev);
	struct net_buf *buf;
	int err = 0;

	rts_ctrl_feed_dout(dev, sizeof(struct usb_setup_packet));

	buf = udc_buf_get(dev, USB_CONTROL_EP_OUT);
	if (buf == NULL) {
		return -ENODATA;
	}

	net_buf_add_mem(buf, (uint8_t *)&priv->setup_pkt, sizeof(priv->setup_pkt));
	udc_ep_buf_set_setup(buf);

	/* Update to next stage of control transfer */
	udc_ctrl_update_stage(dev, buf);

	if (udc_ctrl_stage_is_data_out(dev)) {
		/* Allocate and feed buffer for data OUT stage */
		err = rts_ctrl_feed_dout(dev, udc_data_stage_length(buf));
		if (err == -ENOMEM) {
			err = udc_submit_ep_event(dev, buf, err);
			return err;
		}

		rts_ep0_start_rx(usb);
		rts_ep_set_int_en(usb, EP0, USB_EP0_DATAPKT_RECV_INT, true);
	} else if (udc_ctrl_stage_is_data_in(dev)) {
		/*
		 * Here we have to feed both descriptor tables so that
		 * no setup packets are lost in case of successive
		 * status OUT stage and next setup.
		 */
		/* Finally alloc buffer for IN and submit to upper layer */
		err = udc_ctrl_submit_s_in_status(dev);
	} else { /* no data */
		/*
		 * For all other cases we feed with a buffer
		 * large enough for setup packet.
		 */
		err = udc_ctrl_submit_s_status(dev);
	}

	return err;
}

/* data out event */
static int rts_handle_evt_ctrlout(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	struct net_buf *buf;

	buf = udc_buf_get(dev, ep_cfg->addr);
	if (buf == NULL) {
		return -ENODATA;
	}

	/* Update to next stage of control transfer */
	udc_ctrl_update_stage(dev, buf);

	return udc_ctrl_submit_s_out_status(dev, buf);
}

/* data in event */
static int rts_handle_evt_ctrlin(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;
	struct net_buf *buf;

	buf = udc_buf_peek(dev, ep_cfg->addr);
	if (buf == NULL) {
		LOG_DBG("No buffer for ep 0x%02x", ep_cfg->addr);
		return -ENOBUFS;
	}

	if (udc_ctrl_stage_is_data_in(dev)) {
		if (rts_ep0_start_tx(dev) >= 0) {
			return 0;
		}
	}

	if (udc_ctrl_stage_is_status_in(dev) || udc_ctrl_stage_is_no_data(dev)) {
		/* Start EP0 handshake */
		rts_ep_clr_int_sts(usb, EP0, USB_EP0_CTRL_STATUS_INT);
		rts_ep_set_int_en(usb, EP0, USB_EP0_CTRL_STATUS_INT, true);
		return 0;
	}

	buf = udc_buf_get(dev, ep_cfg->addr);

	/* Update to next stage of control transfer */
	udc_ctrl_update_stage(dev, buf);

	if (udc_ctrl_stage_is_status_out(dev)) {
		/*
		 * IN transfer finished, release buffer,
		 * control OUT buffer should be already fed.
		 * Start EP0 handshake
		 */
		net_buf_unref(buf);
		rts_ctrl_feed_dout(dev, 0);
		rts_ep_clr_int_sts(usb, EP0, USB_EP0_CTRL_STATUS_INT);
		rts_ep_set_int_en(usb, EP0, USB_EP0_CTRL_STATUS_INT, true);
	}

	return 0;
}

static int rts_handle_evt_sts(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	struct net_buf *buf;

	buf = udc_buf_get(dev, ep_cfg->addr);
	if (buf == NULL) {
		LOG_DBG("No buffer for ep 0x%02x", ep_cfg->addr);
		return -ENOBUFS;
	}

	/* Status stage finished, notify upper layer */
	udc_ctrl_submit_status(dev, buf);

	/* Update to next stage of control transfer */
	udc_ctrl_update_stage(dev, buf);

	return 0;
}

/* endpoint event */
static void rts_prepare_tx(const struct device *dev, struct udc_ep_config *const ep_cfg,
			   struct net_buf *buf)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;
	uint8_t ep_idx = rts_ep_idx(ep_cfg->addr);
	uint16_t trans_len = 0;
	uint8_t *data = NULL;

	if ((ep_cfg->attributes & USB_EP_TRANSFER_TYPE_MASK) == USB_EP_TYPE_BULK) {
		rts_bulkin(usb, ep_idx, buf->data, buf->len);
	} else {
		trans_len = MIN(buf->len, rts_ep_get_mps(usb, ep_idx));
		data = net_buf_pull_mem(buf, trans_len);
		rts_intin(usb, ep_idx, data, trans_len);
	}
}

static void rts_prepare_rx(const struct device *dev, struct udc_ep_config *const ep_cfg,
			   struct net_buf *buf)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;
	struct udc_rts_data *const priv = udc_get_private(dev);
	uint8_t ep_idx = rts_ep_idx(ep_cfg->addr);
	uint8_t idx;
	uint16_t mps = rts_ep_get_mps(usb, ep_idx);
	uint16_t bytecnt;
	struct rts_drv_event evt;

	if ((ep_cfg->attributes & USB_EP_TRANSFER_TYPE_MASK) == USB_EP_TYPE_BULK) {
		if (ep_idx == EPA) {
			idx = 0;
		} else {
			idx = 1;
		}

		if (priv->bulkout[idx].last_sp) {
			rts_ep_fifo_flush(usb, ep_idx);
			rts_ep_clr_int_sts(usb, ep_idx,
					   USB_BULKOUT_SHORTPKT_RECV_INT |
						   USB_BULKOUT_DATAPKT_RECV_INT);
			priv->bulkout[idx].last_sp = false;
		}

		if (rts_ep_get_int_sts(usb, ep_idx, USB_BULKOUT_SHORTPKT_RECV_INT)) {
			bytecnt = rts_ep_rx_bc(usb, ep_idx);
			if (bytecnt >= mps) {
				priv->bulkout[idx].recv_len = mps;
			} else {
				priv->bulkout[idx].last_sp = true;
				if (bytecnt == 0) {
					evt.ep = ep_cfg->addr;
					evt.evt_type = EVT_DONE;
					k_msgq_put(&drv_msgq, &evt, K_NO_WAIT);
					return;
				}
				priv->bulkout[idx].recv_len = bytecnt;
			}
			rts_bulkout_startdma(usb, ep_idx, net_buf_tail(buf),
					     priv->bulkout[idx].recv_len);
			rts_mc_set_int_en(usb, ep_idx, EP_MC_INT_DMA_DONE, true);
		} else {
			if (rts_ep_rx_bc(usb, ep_idx) == mps) {
				rts_ep_clr_int_sts(usb, ep_idx, USB_BULKOUT_DATAPKT_RECV_INT);
				priv->bulkout[idx].recv_len = mps;
				rts_bulkout_startdma(usb, ep_idx, net_buf_tail(buf),
						     priv->bulkout[idx].recv_len);
				rts_mc_set_int_en(usb, ep_idx, EP_MC_INT_DMA_DONE, true);
			} else {
				rts_ep_set_int_en(usb, ep_idx, USB_BULKOUT_DATAPKT_RECV_INT, true);
			}
		}
	} else {
		rts_ep_set_int_en(usb, ep_idx, USB_INTOUT_DATAPKT_RECV_INT, true);
	}
}

static int rts_handle_evt_done(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	struct net_buf *buf;

	buf = udc_buf_get(dev, ep_cfg->addr);
	if (buf == NULL) {
		return -ENOBUFS;
	}

	udc_ep_set_busy(dev, ep_cfg->addr, false);

	return udc_submit_ep_event(dev, buf, 0);
}

static int rts_handle_evt_dout(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	struct net_buf *buf;

	buf = udc_buf_peek(dev, ep_cfg->addr);
	if (buf == NULL) {
		return 0;
	}

	rts_prepare_rx(dev, ep_cfg, buf);
	return 0;
}

static int rts_handle_evt_din(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	struct net_buf *buf;

	buf = udc_buf_peek(dev, ep_cfg->addr);
	if (buf == NULL) {
		return 0;
	}

	rts_prepare_tx(dev, ep_cfg, buf);
	return 0;
}

static int rts_handle_xfer_next(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	struct net_buf *buf;

	buf = udc_buf_peek(dev, ep_cfg->addr);
	if (buf == NULL) {
		return 0;
	}

	udc_ep_set_busy(dev, ep_cfg->addr, true);

	if (USB_EP_DIR_IS_OUT(ep_cfg->addr)) {
		rts_prepare_rx(dev, ep_cfg, buf);
	} else {
		rts_prepare_tx(dev, ep_cfg, buf);
	}

	return 0;
}

/*
 * You can use one thread per driver instance model or UDC driver workqueue,
 * whichever model suits your needs best. If you decide to use the UDC workqueue,
 * enable Kconfig option UDC_WORKQUEUE and remove the handler below and
 * caller from the UDC_SKELETON_DEVICE_DEFINE macro.
 */
static ALWAYS_INLINE void rts_thread_handler(void *const arg)
{
	const struct device *dev = (const struct device *)arg;
	struct udc_ep_config *ep_cfg;
	struct rts_drv_event evt;
	int err = 0;

	/* This is the bottom-half of the ISR handler and the place where
	 * a new transfer can be fed.
	 */
	k_msgq_get(&drv_msgq, &evt, K_FOREVER);
	LOG_DBG("Driver %p thread started %d", dev, evt.evt_type);
	ep_cfg = udc_get_ep_cfg(dev, evt.ep);

	switch (evt.evt_type) {
	case EVT_SETUP:
		err = rts_handle_evt_setup(dev);
		break;
	case EVT_CTRL_DIN:
		err = rts_handle_evt_ctrlin(dev, ep_cfg);
		break;
	case EVT_CTRL_DOUT:
		err = rts_handle_evt_ctrlout(dev, ep_cfg);
		break;
	case EVT_STATUS:
		err = rts_handle_evt_sts(dev, ep_cfg);
		break;
	case EVT_XFER:
		break;
	case EVT_DOUT:
		err = rts_handle_evt_dout(dev, ep_cfg);
		break;
	case EVT_DIN:
		err = rts_handle_evt_din(dev, ep_cfg);
		break;
	case EVT_DONE:
		err = rts_handle_evt_done(dev, ep_cfg);
		break;
	default:
		break;
	}

	if (!udc_ep_is_busy(dev, ep_cfg->addr) && (USB_EP_GET_IDX(ep_cfg->addr) != 0)) {
		err = rts_handle_xfer_next(dev, ep_cfg);
	}

	if (err) {
		udc_submit_event(dev, UDC_EVT_ERROR, err);
	}
}

static void rts_ctrl_handle_setup(const struct device *dev)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;
	struct udc_rts_data *const priv = udc_get_private(dev);
	struct rts_drv_event evt = {
		.evt_type = EVT_SETUP,
		.ep = USB_CONTROL_EP_OUT,
	};

	rts_ep_clr_int_sts(usb, EP0, USB_EP0_SETUP_PACKET_INT);
	rts_ep0_get_setup_pkt(usb, (uint8_t *)&priv->setup_pkt, sizeof(struct usb_setup_packet));

	k_msgq_put(&drv_msgq, &evt, K_NO_WAIT);
}

static void rts_ctrl_handle_trans(const struct device *dev)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;
	struct rts_drv_event evt = {
		.evt_type = EVT_CTRL_DIN,
		.ep = USB_CONTROL_EP_IN,
	};

	rts_ep_clr_int_sts(usb, EP0, USB_EP0_DATAPKT_TRANS_INT);

	rts_ep_set_int_en(usb, EP0, USB_EP0_DATAPKT_TRANS_INT, false);

	k_msgq_put(&drv_msgq, &evt, K_NO_WAIT);
}

static void rts_ctrl_handle_recv(const struct device *dev)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;
	uint16_t transfer_length;
	struct net_buf *buf;
	struct rts_drv_event evt = {
		.evt_type = EVT_CTRL_DOUT,
		.ep = USB_CONTROL_EP_OUT,
	};
	uint8_t data[USB_EP0_MPS] = {0};

	buf = udc_buf_peek(dev, USB_CONTROL_EP_OUT);
	if (buf == NULL) {
		return;
	}

	rts_ep_clr_int_sts(usb, EP0, USB_EP0_DATAPKT_RECV_INT);

	transfer_length = rts_ctrl_handle_recv_sub(usb, data, USB_EP0_MPS);

	if (transfer_length < 0) {
		return;
	}

	net_buf_add_mem(buf, data, transfer_length);

	if (net_buf_tailroom(buf)) {
		rts_ep0_start_rx(usb);
	} else {
		rts_ep_set_int_en(usb, EP0, USB_EP0_DATAPKT_RECV_INT, false);
		k_msgq_put(&drv_msgq, &evt, K_NO_WAIT);
	}
}

static void rts_ctrl_handle_status(const struct device *dev)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;

	struct rts_drv_event evt;

	rts_ep0_hsk(usb);

	if (udc_ctrl_stage_is_status_out(dev)) {
		evt.ep = USB_CONTROL_EP_OUT;
	} else {
		evt.ep = USB_CONTROL_EP_IN;
	}

	evt.evt_type = EVT_STATUS;
	k_msgq_put(&drv_msgq, &evt, K_NO_WAIT);
	rts_ep_set_int_en(usb, EP0, USB_EP0_CTRL_STATUS_INT, false);
	rts_ep_clr_int_sts(usb, EP0, USB_EP0_CTRL_STATUS_INT);
}

static void rts_bulk_handle_recv(const struct device *dev, uint8_t ep_idx)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;
	struct udc_rts_data *const priv = udc_get_private(dev);
	struct net_buf *buf;
	uint8_t idx;
	uint16_t mps = rts_ep_get_mps(usb, ep_idx);
	struct rts_drv_event evt = {
		.ep = USB_EP_DIR_OUT | ep_idx,
		.evt_type = EVT_DONE,
	};
	uint16_t bytecnt;

	buf = udc_buf_peek(dev, ep_idx);
	if (buf == NULL) {
		return;
	}

	if (ep_idx == EPA) {
		idx = 0;
	} else {
		idx = 1;
	}

	priv->bulkout[idx].recv_len = mps;

	bytecnt = rts_ep_rx_bc(usb, ep_idx);
	if (bytecnt == mps) {
		rts_ep_clr_int_sts(usb, ep_idx, USB_BULKOUT_DATAPKT_RECV_INT);
	} else if (bytecnt > mps) {
		/* receive one pkt */
	} else {
		if (WAIT_FOR((rts_ep_rx_bc(usb, ep_idx) >= mps) ||
				     rts_ep_get_int_sts(usb, ep_idx, USB_BULKOUT_SHORTPKT_RECV_INT),
			     1000, k_busy_wait(1))) {
			bytecnt = rts_ep_rx_bc(usb, ep_idx);
			if (bytecnt == mps) {
				rts_ep_clr_int_sts(usb, ep_idx, USB_BULKOUT_DATAPKT_RECV_INT);
			} else if (bytecnt < mps) {
				priv->bulkout[idx].last_sp = true;
				rts_ep_clr_int_sts(usb, ep_idx, USB_BULKOUT_DATAPKT_RECV_INT);
				priv->bulkout[idx].recv_len = bytecnt;
			} else {
				/* bytecnt > mps, receive one pkt */
			}
		} else {
			udc_submit_ep_event(dev, buf, -ETIMEDOUT);
			return;
		}
	}

	if (priv->bulkout[idx].recv_len) {
		rts_bulkout_startdma(usb, ep_idx, net_buf_tail(buf), priv->bulkout[idx].recv_len);
		rts_mc_set_int_en(usb, ep_idx, EP_MC_INT_DMA_DONE, true);
	} else {
		k_msgq_put(&drv_msgq, &evt, K_NO_WAIT);
	}

	rts_ep_set_int_en(usb, ep_idx, USB_BULKOUT_DATAPKT_RECV_INT, false);
}

static void rts_bulkout_handle_done(const struct device *dev, uint8_t ep_idx)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;
	struct udc_rts_data *const priv = udc_get_private(dev);
	struct net_buf *buf;
	uint8_t idx;
	struct rts_drv_event evt = {
		.ep = USB_EP_DIR_OUT | ep_idx,
	};

	buf = udc_buf_peek(dev, ep_idx);
	if (buf == NULL) {
		return;
	}

	if (ep_idx == EPA) {
		idx = 0;
	} else {
		idx = 1;
	}

	rts_bulk_stopdma(usb, ep_idx);

	buf->len += priv->bulkout[idx].recv_len;

	rts_mc_clr_int_sts(usb, ep_idx, EP_MC_INT_DMA_DONE);
	rts_mc_set_int_en(usb, ep_idx, EP_MC_INT_DMA_DONE, false);
	if (net_buf_tailroom(buf) && (priv->bulkout[idx].recv_len == rts_ep_get_mps(usb, ep_idx))) {
		evt.evt_type = EVT_DOUT;
		k_msgq_put(&drv_msgq, &evt, K_NO_WAIT);
	} else {
		evt.evt_type = EVT_DONE;
		k_msgq_put(&drv_msgq, &evt, K_NO_WAIT);
	}
}

static void rts_bulk_handle_transzlp(const struct device *dev, uint8_t ep_idx)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;
	struct rts_drv_event evt = {
		.ep = USB_EP_DIR_IN | ep_idx,
		.evt_type = EVT_DONE,
	};

	rts_ep_clr_int_sts(usb, ep_idx, USB_BULKIN_TRANS_END_INT);
	rts_ep_set_int_en(usb, ep_idx, USB_BULKIN_TRANS_END_INT, false);

	k_msgq_put(&drv_msgq, &evt, K_NO_WAIT);
}

static void rts_bulkin_handle_dmadone(const struct device *dev, uint8_t ep_idx)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;

	rts_mc_clr_int_sts(usb, ep_idx, EP_MC_INT_DMA_DONE);
	rts_mc_set_int_en(usb, ep_idx, EP_MC_INT_DMA_DONE, false);
	rts_mc_set_int_en(usb, ep_idx, EP_MC_INT_SIE_DONE, true);
}

static void rts_bulkin_handle_siedone(const struct device *dev, uint8_t ep_idx)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;
	struct rts_drv_event evt = {
		.ep = USB_EP_DIR_IN | ep_idx,
		.evt_type = EVT_DONE,
	};

	rts_mc_clr_int_sts(usb, ep_idx, EP_MC_INT_SIE_DONE);

	rts_bulk_stopdma(usb, ep_idx);

	rts_mc_set_int_en(usb, ep_idx, EP_MC_INT_SIE_DONE, false);

	k_msgq_put(&drv_msgq, &evt, K_NO_WAIT);
}

static void rts_int_handle_trans(const struct device *dev, uint8_t ep_idx)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;
	struct rts_drv_event evt = {
		.ep = USB_EP_DIR_IN | ep_idx,
	};
	struct net_buf *buf = udc_buf_peek(dev, evt.ep);

	if (buf == NULL) {
		return;
	}

	if (buf->len) {
		evt.evt_type = EVT_DIN;
		k_msgq_put(&drv_msgq, &evt, K_NO_WAIT);
	} else {
		evt.evt_type = EVT_DONE;
		k_msgq_put(&drv_msgq, &evt, K_NO_WAIT);
	}

	rts_ep_clr_int_sts(usb, ep_idx, USB_INTIN_DATAPKT_TRANS_INT);
	rts_ep_set_int_en(usb, ep_idx, USB_INTIN_DATAPKT_TRANS_INT, false);
}

static void rts_int_handle_recv(const struct device *dev, uint8_t ep_idx)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;
	struct net_buf *buf;
	struct rts_drv_event evt = {
		.ep = USB_EP_DIR_OUT | ep_idx,
		.evt_type = EVT_DONE,
	};

	buf = udc_buf_peek(dev, evt.ep);
	if (buf == NULL) {
		return;
	}

	buf->len = rts_intout(usb, ep_idx, buf->data);

	rts_ep_set_int_en(usb, ep_idx, USB_INTOUT_DATAPKT_RECV_INT, false);
	k_msgq_put(&drv_msgq, &evt, K_NO_WAIT);
}

static void rts_usb_dc_isr(const struct device *dev)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;

	if (rts_ls_get_int_en(usb, USB_LS_PORT_RST_INT) &&
	    rts_ls_get_int_sts(usb, USB_LS_PORT_RST_INT)) {
		rts_usb_handle_rst(dev);
	}

	if (rts_ep_get_int_en(usb, EP0, USB_EP0_SETUP_PACKET_INT) &&
	    rts_ep_get_int_sts(usb, EP0, USB_EP0_SETUP_PACKET_INT)) {
		rts_ctrl_handle_setup(dev);
	}

	if (rts_ep_get_int_en(usb, EP0, USB_EP0_DATAPKT_TRANS_INT) &&
	    rts_ep_get_int_sts(usb, EP0, USB_EP0_DATAPKT_TRANS_INT)) {
		rts_ctrl_handle_trans(dev);
	}

	if (rts_ep_get_int_en(usb, EP0, USB_EP0_DATAPKT_RECV_INT) &&
	    rts_ep_get_int_sts(usb, EP0, USB_EP0_DATAPKT_RECV_INT)) {
		rts_ctrl_handle_recv(dev);
	}

	if (rts_ep_get_int_en(usb, EP0, USB_EP0_CTRL_STATUS_INT) &&
	    rts_ep_get_int_sts(usb, EP0, USB_EP0_CTRL_STATUS_INT)) {
		rts_ctrl_handle_status(dev);
	}

	/* Other endpoints */
	if (rts_ep_get_int_en(usb, EPA, USB_BULKOUT_DATAPKT_RECV_INT) &&
	    rts_ep_get_int_sts(usb, EPA, USB_BULKOUT_DATAPKT_RECV_INT)) {
		rts_bulk_handle_recv(dev, EPA);
	}

	if (rts_ep_get_int_en(usb, EPB, USB_BULKIN_TRANS_END_INT) &&
	    rts_ep_get_int_sts(usb, EPB, USB_BULKIN_TRANS_END_INT)) {
		rts_bulk_handle_transzlp(dev, EPB);
	}

	if (rts_ep_get_int_en(usb, EPE, USB_BULKOUT_DATAPKT_RECV_INT) &&
	    rts_ep_get_int_sts(usb, EPE, USB_BULKOUT_DATAPKT_RECV_INT)) {
		rts_bulk_handle_recv(dev, EPE);
	}

	if (rts_ep_get_int_en(usb, EPF, USB_BULKIN_TRANS_END_INT) &&
	    rts_ep_get_int_sts(usb, EPF, USB_BULKIN_TRANS_END_INT)) {
		rts_bulk_handle_transzlp(dev, EPF);
	}

	if (rts_ep_get_int_en(usb, EPC, USB_INTIN_DATAPKT_TRANS_INT) &&
	    rts_ep_get_int_sts(usb, EPC, USB_INTIN_DATAPKT_TRANS_INT)) {
		rts_int_handle_trans(dev, EPC);
	}

	if (rts_ep_get_int_en(usb, EPD, USB_INTIN_DATAPKT_TRANS_INT) &&
	    rts_ep_get_int_sts(usb, EPD, USB_INTIN_DATAPKT_TRANS_INT)) {
		rts_int_handle_trans(dev, EPD);
	}

	if (rts_ep_get_int_en(usb, EPG, USB_INTOUT_DATAPKT_RECV_INT) &&
	    rts_ep_get_int_sts(usb, EPG, USB_INTOUT_DATAPKT_RECV_INT)) {
		rts_int_handle_recv(dev, EPG);
	}

	if (rts_ls_get_int_en(usb, USB_LS_SUSPEND_INT) &&
	    rts_ls_get_int_sts(usb, USB_LS_SUSPEND_INT)) {
		rts_usb_handle_suspend(dev);
	}

	if (rts_ls_get_int_en(usb, USB_LS_RESUME_INT) &&
	    rts_ls_get_int_sts(usb, USB_LS_RESUME_INT)) {
		rts_usb_handle_resume(dev);
	}

	if (rts_ls_get_int_en(usb, USB_LS_SOF_INT) && rts_ls_get_int_sts(usb, USB_LS_SOF_INT)) {
		rts_usb_handle_sof(dev);
	}
}

static void rts_usb_dc_isr_mc(const struct device *dev)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;

	if (rts_mc_get_int_en(usb, EPA, EP_MC_INT_DMA_DONE) &&
	    rts_mc_get_int_sts(usb, EPA, EP_MC_INT_DMA_DONE)) {
		rts_bulkout_handle_done(dev, EPA);
	}

	if (rts_mc_get_int_en(usb, EPE, EP_MC_INT_DMA_DONE) &&
	    rts_mc_get_int_sts(usb, EPE, EP_MC_INT_DMA_DONE)) {
		rts_bulkout_handle_done(dev, EPE);
	}

	if (rts_mc_get_int_en(usb, EPB, EP_MC_INT_DMA_DONE) &&
	    rts_mc_get_int_sts(usb, EPB, EP_MC_INT_DMA_DONE)) {
		rts_bulkin_handle_dmadone(dev, EPB);
	}

	if (rts_mc_get_int_en(usb, EPF, EP_MC_INT_DMA_DONE) &&
	    rts_mc_get_int_sts(usb, EPF, EP_MC_INT_DMA_DONE)) {
		rts_bulkin_handle_dmadone(dev, EPF);
	}

	if (rts_mc_get_int_en(usb, EPB, EP_MC_INT_SIE_DONE) &&
	    rts_mc_get_int_sts(usb, EPB, EP_MC_INT_SIE_DONE)) {
		rts_bulkin_handle_siedone(dev, EPB);
	}

	if (rts_mc_get_int_en(usb, EPF, EP_MC_INT_SIE_DONE) &&
	    rts_mc_get_int_sts(usb, EPF, EP_MC_INT_SIE_DONE)) {
		rts_bulkin_handle_siedone(dev, EPF);
	}
}

/*
 * This is called in the context of udc_ep_enqueue() and must
 * not block. The driver can immediately claim the buffer if the queue is empty,
 * but usually it is offloaded to a thread or workqueue to handle transfers
 * in a single location. Please refer to existing driver implementations
 * for examples.
 */
static int udc_rts_ep_enqueue(const struct device *dev, struct udc_ep_config *const ep_cfg,
			      struct net_buf *buf)
{
	LOG_DBG("%p enqueue %p, 0x%x", dev, buf, ep_cfg->addr);
	struct rts_drv_event evt = {
		.ep = ep_cfg->addr,
	};
	udc_buf_put(ep_cfg, buf);

	if (ep_cfg->addr == USB_CONTROL_EP_IN) {
		evt.evt_type = EVT_CTRL_DIN;
	} else {
		evt.evt_type = EVT_XFER;
	}

	if (!ep_cfg->stat.halted) {
		/*
		 * It is fine to enqueue a transfer for a halted endpoint,
		 * you need to make sure that transfers are retriggered when
		 * the halt is cleared.
		 *
		 * Always use the abbreviation 'ep' for the endpoint address
		 * and 'ep_idx' or 'ep_num' for the endpoint number identifiers.
		 * Although struct udc_ep_config uses address to be unambiguous
		 * in its context.
		 */
		k_msgq_put(&drv_msgq, &evt, K_NO_WAIT);
	} else {
		LOG_DBG("Enable ep 0x%02x", ep_cfg->addr);
	}

	return 0;
}

/*
 * This is called in the context of udc_ep_dequeue()
 * and must remove all requests from an endpoint queue
 * Successful removal should be reported to the higher level with
 * ECONNABORTED as the request result.
 * It is up to the request owner to clean up or reuse the buffer.
 */
static int udc_rts_ep_dequeue(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	unsigned int lock_key;
	struct net_buf *buf;

	lock_key = irq_lock();

	buf = udc_buf_get_all(dev, ep_cfg->addr);
	if (buf) {
		udc_submit_ep_event(dev, buf, -ECONNABORTED);
	}

	udc_ep_set_busy(dev, ep_cfg->addr, false);

	irq_unlock(lock_key);

	return 0;
}

/*
 * Configure and make an endpoint ready for use.
 * This is called in the context of udc_ep_enable() or udc_ep_enable_internal(),
 * the latter of which may be used by the driver to enable control endpoints.
 */
static int udc_rts_ep_enable(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	uint8_t ep_idx = rts_ep_idx(ep_cfg->addr);

	LOG_DBG("Enable ep 0x%02x", ep_cfg->addr);
	rts_ep_en(dev, ep_idx, ep_cfg);

	return 0;
}

/*
 * Opposite function to udc_rts_ep_enable(). udc_ep_disable_internal()
 * may be used by the driver to disable control endpoints.
 */
static int udc_rts_ep_disable(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;

	uint8_t ep_idx = rts_ep_idx(ep_cfg->addr);
	unsigned int lock_key;

	LOG_DBG("Disable ep 0x%02x", ep_cfg->addr);

	lock_key = irq_lock();

	if ((ep_cfg->attributes & USB_EP_TRANSFER_TYPE_MASK) == USB_EP_TYPE_BULK) {
		rts_bulk_stopdma(usb, ep_idx);
		rts_ep_fifo_flush(usb, ep_idx);

		rts_mc_clr_int_sts(usb, ep_idx, 0xff);
		rts_mc_set_int_en(usb, ep_idx, 0xff, false);
	}
	rts_ep_clr_int_sts(usb, ep_idx, 0xff);
	rts_ep_set_int_en(usb, ep_idx, 0xff, false);

	irq_unlock(lock_key);

	rts_ep_cfg_en(usb, ep_idx, false);

	if (ep_cfg->addr == USB_CONTROL_EP_OUT) {
		struct net_buf *buf = udc_buf_get_all(dev, ep_cfg->addr);

		if (buf) {
			net_buf_unref(buf);
		}
	}

	udc_ep_set_busy(dev, ep_cfg->addr, false);

	return 0;
}

/* Halt endpoint. Halted endpoint should respond with a STALL handshake. */
static int udc_rts_ep_set_halt(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;
	uint8_t ep_idx = rts_ep_idx(ep_cfg->addr);

	LOG_DBG("Set halt ep 0x%02x", ep_cfg->addr);

	if (ep_idx != EP0) {
		ep_cfg->stat.halted = true;
	}

	rts_ep_stall(usb, ep_idx, true);

	return 0;
}

/*
 * Opposite to halt endpoint. If there are requests in the endpoint queue,
 * the next transfer should be prepared.
 */
static int udc_rts_ep_clear_halt(const struct device *dev, struct udc_ep_config *const ep_cfg)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;
	uint8_t ep_idx = rts_ep_idx(ep_cfg->addr);
	struct rts_drv_event evt = {
		.ep = ep_cfg->addr,
		.evt_type = EVT_XFER,
	};

	LOG_DBG("Clear halt ep 0x%02x", ep_cfg->addr);

	ep_cfg->stat.halted = false;

	if ((ep_cfg->attributes & USB_EP_TRANSFER_TYPE_MASK) == USB_EP_TYPE_BULK) {
		rts_mc_clr_int_sts(usb, ep_idx, 0xFF);
		rts_ep_fifo_flush(usb, ep_idx);
	}

	rts_ep_clr_int_sts(usb, ep_idx, 0xFF);
	rts_ep_force_toggle(usb, ep_idx, USB_EP_DATA_ID_DATA0);
	rts_ep_stall(usb, ep_idx, false);

	/* Resume queued transfers if any */
	if (udc_buf_peek(dev, ep_cfg->addr)) {
		k_msgq_put(&drv_msgq, &evt, K_NO_WAIT);
	}

	return 0;
}

static int udc_rts_set_address(const struct device *dev, const uint8_t addr)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;
	struct udc_data *data = dev->data;

	LOG_DBG("Set new address %u for %p", addr, dev);

	rts_usb_set_devaddr(usb, addr);

	if (rts_usb_get_devaddr(usb) != 0) {
		data->caps.addr_before_status = false;
	}

	return 0;
}

static int udc_rts_host_wakeup(const struct device *dev)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;

	LOG_DBG("Remote wakeup from %p", dev);
	rts_usb_trg_rsumK(usb);
	return 0;
}

/* Return actual USB device speed */
static enum udc_bus_speed udc_rts_device_speed(const struct device *dev)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;

	return rts_usb_test_hs(usb) ? UDC_BUS_SPEED_HS : UDC_BUS_SPEED_FS;
}

static int udc_rts_enable(const struct device *dev)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;

	LOG_DBG("Enable device %p", dev);

	if (cfg->speed_idx == 2) {
		rts_usb_connect_hs(usb);
	} else {
		rts_usb_connect_fs(usb);
	}

	cfg->irq_enable_func(dev);

	return 0;
}

static int udc_rts_disable(const struct device *dev)
{
	const struct udc_rts_config *cfg = dev->config;
	const USB_TypeDef *usb = &cfg->usb;

	LOG_DBG("Enable device %p", dev);

	rts_usb_disconnect(usb);

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_OUT)) {
		LOG_ERR("Failed to disable control endpoint");
		return -EIO;
	}

	if (udc_ep_disable_internal(dev, USB_CONTROL_EP_IN)) {
		LOG_ERR("Failed to disable control endpoint");
		return -EIO;
	}

	cfg->irq_disable_func(dev);

	return 0;
}

/*
 * Prepare and configure most of the parts, if the controller has a way
 * of detecting VBUS activity it should be enabled here.
 * Only udc_rts_enable() makes device visible to the host.
 */
static int udc_rts_init(const struct device *dev)
{
	rts_usb_init(dev, true);

	if (udc_ep_enable_internal(dev, USB_CONTROL_EP_OUT, USB_EP_TYPE_CONTROL, 64, 0)) {
		LOG_ERR("Failed to enable control endpoint");
		return -EIO;
	}

	if (udc_ep_enable_internal(dev, USB_CONTROL_EP_IN, USB_EP_TYPE_CONTROL, 64, 0)) {
		LOG_ERR("Failed to enable control endpoint");
		return -EIO;
	}

	return 0;
}

/* Shut down the controller completely */
static int udc_rts_shutdown(const struct device *dev)
{
	const struct udc_rts_config *cfg = dev->config;

	/* TODO */
	cfg->irq_disable_func(dev);

	return 0;
}

/*
 * This is called once to initialize the controller and endpoints
 * capabilities, and register endpoint structures.
 */
static int udc_rts_driver_preinit(const struct device *dev)
{
	const struct udc_rts_config *cfg = dev->config;
	struct udc_data *data = dev->data;
	int err;

	/*
	 * You do not need to initialize it if your driver does not use
	 * udc_lock_internal() / udc_unlock_internal(), but implements its
	 * own mechanism.
	 */
	k_mutex_init(&data->mutex);

	data->caps.rwup = true;
	data->caps.addr_before_status = true;
	data->caps.out_ack = false;
	data->caps.mps0 = UDC_MPS0_64;
	if (cfg->speed_idx == 2) {
		data->caps.hs = true;
	}

	/* config->num_of_eps */
	for (int i = 0, n = 0; i < MAX_NUM_EP; i++) {
		if (i == 0) {
			cfg->ep_cfg_out[n].caps.control = 1;
			cfg->ep_cfg_out[n].caps.mps = 64;
		} else {
			if (ep_dir_table[i] != USB_EP_DIR_OUT) {
				continue;
			} else {
				if (ep_lnum_table[i] == EPG) {
					cfg->ep_cfg_out[n].caps.interrupt = 1;
					cfg->ep_cfg_out[n].caps.mps = USB_INT_EPG_MPS;
				} else {
					cfg->ep_cfg_out[n].caps.bulk = 1;
					if (data->caps.hs) {
						cfg->ep_cfg_out[n].caps.mps = USB_BULK_MPS_HS;
					} else {
						cfg->ep_cfg_out[n].caps.mps = USB_BULK_MPS_FS;
					}
				}
			}
		}
		cfg->ep_cfg_out[n].caps.out = 1;
		cfg->ep_cfg_out[n].caps.in = 0;
		cfg->ep_cfg_out[n].addr = USB_EP_DIR_OUT | ep_lnum_table[i];
		err = udc_register_ep(dev, &cfg->ep_cfg_out[n]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
		n += 1;
	}

	for (int i = 0, n = 0; i < MAX_NUM_EP; i++) {
		if (i == 0) {
			cfg->ep_cfg_in[n].caps.control = 1;
			cfg->ep_cfg_in[n].caps.mps = 64;
		} else {
			if (ep_dir_table[i] != USB_EP_DIR_IN) {
				continue;
			} else {
				if ((ep_lnum_table[i] == EPC) || (ep_lnum_table[i] == EPD)) {
					cfg->ep_cfg_in[n].caps.interrupt = 1;
					if (ep_lnum_table[i] == EPD) {
						cfg->ep_cfg_in[n].caps.mps = USB_INT_EPD_MPS;
					} else {
						cfg->ep_cfg_in[n].caps.mps =
							data->caps.hs ? USB_INT_EPC_MPS_HS
								      : USB_INT_EPC_MPS_FS;
					}
				} else {
					cfg->ep_cfg_in[n].caps.bulk = 1;
					cfg->ep_cfg_in[n].caps.mps =
						data->caps.hs ? USB_BULK_MPS_HS : USB_BULK_MPS_FS;
				}
			}
		}
		cfg->ep_cfg_in[n].caps.out = 0;
		cfg->ep_cfg_in[n].caps.in = 1;
		cfg->ep_cfg_in[n].addr = USB_EP_DIR_IN | ep_lnum_table[i];
		err = udc_register_ep(dev, &cfg->ep_cfg_in[n]);
		if (err != 0) {
			LOG_ERR("Failed to register endpoint");
			return err;
		}
		n += 1;
	}

	cfg->make_thread(dev);
	LOG_INF("Device %p (max. speed %d)", dev, cfg->speed_idx);

	return 0;
}

static int udc_rts_lock(const struct device *dev)
{
	return udc_lock_internal(dev, K_FOREVER);
}

static int udc_rts_unlock(const struct device *dev)
{
	return udc_unlock_internal(dev);
}

/*
 * UDC API structure.
 * Note, you do not need to implement basic checks, these are done by
 * the UDC common layer udc_common.c
 */
static const struct udc_api udc_rts_api = {
	.lock = udc_rts_lock,
	.unlock = udc_rts_unlock,
	.device_speed = udc_rts_device_speed,
	.init = udc_rts_init,
	.enable = udc_rts_enable,
	.disable = udc_rts_disable,
	.shutdown = udc_rts_shutdown,
	.set_address = udc_rts_set_address,
	.host_wakeup = udc_rts_host_wakeup,
	.ep_enable = udc_rts_ep_enable,
	.ep_disable = udc_rts_ep_disable,
	.ep_set_halt = udc_rts_ep_set_halt,
	.ep_clear_halt = udc_rts_ep_clear_halt,
	.ep_enqueue = udc_rts_ep_enqueue,
	.ep_dequeue = udc_rts_ep_dequeue,
};

/*
 * A UDC driver should always be implemented as a multi-instance
 * driver, even if your platform does not require it.
 */
#define UDC_RTS5816_DEVICE_DEFINE(n)                                                               \
	K_THREAD_STACK_DEFINE(udc_rts_stack_##n, CONFIG_UDC_RTS5816_STACK_SIZE);                   \
                                                                                                   \
	static void udc_rts_thread_##n(void *dev, void *arg1, void *arg2)                          \
	{                                                                                          \
		while (true) {                                                                     \
			rts_thread_handler(dev);                                                   \
		}                                                                                  \
	}                                                                                          \
                                                                                                   \
	static void udc_rts_make_thread_##n(const struct device *dev)                              \
	{                                                                                          \
		struct udc_rts_data *priv = udc_get_private(dev);                                  \
                                                                                                   \
		k_thread_create(&priv->thread_data, udc_rts_stack_##n,                             \
				K_THREAD_STACK_SIZEOF(udc_rts_stack_##n), udc_rts_thread_##n,      \
				(void *)dev, NULL, NULL,                                           \
				K_PRIO_COOP(CONFIG_UDC_RTS5816_THREAD_PRIORITY), K_ESSENTIAL,      \
				K_NO_WAIT);                                                        \
		k_thread_name_set(&priv->thread_data, dev->name);                                  \
	}                                                                                          \
                                                                                                   \
	static void udc_rts_irq_enable_func_##n(const struct device *dev)                          \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN_BY_IDX(n, 0), DT_INST_IRQ_BY_IDX(n, 0, priority),         \
			    rts_usb_dc_isr, DEVICE_DT_INST_GET(n), 0);                             \
                                                                                                   \
		irq_enable(DT_INST_IRQN_BY_IDX(n, 0));                                             \
                                                                                                   \
		IRQ_CONNECT(DT_INST_IRQN_BY_IDX(n, 1), DT_INST_IRQ_BY_IDX(n, 1, priority),         \
			    rts_usb_dc_isr_mc, DEVICE_DT_INST_GET(n), 0);                          \
                                                                                                   \
		irq_enable(DT_INST_IRQN_BY_IDX(n, 1));                                             \
	}                                                                                          \
                                                                                                   \
	static void udc_rts_irq_disable_func_##n(const struct device *dev)                         \
	{                                                                                          \
		if (irq_is_enabled(DT_INST_IRQN_BY_IDX(n, 0))) {                                   \
			irq_disable(DT_INST_IRQN_BY_IDX(n, 0));                                    \
		}                                                                                  \
                                                                                                   \
		if (irq_is_enabled(DT_INST_IRQN_BY_IDX(n, 1))) {                                   \
			irq_disable(DT_INST_IRQN_BY_IDX(n, 1));                                    \
		}                                                                                  \
	}                                                                                          \
                                                                                                   \
	static struct udc_ep_config ep_cfg_out[DT_INST_PROP(n, num_out_endpoints)];                \
	static struct udc_ep_config ep_cfg_in[DT_INST_PROP(n, num_in_endpoints)];                  \
                                                                                                   \
	static const struct udc_rts_config udc_rts_config_##n = {                                  \
		.num_of_eps = DT_INST_PROP(n, num_endpoints),                                      \
		.ep_cfg_in = ep_cfg_in,                                                            \
		.ep_cfg_out = ep_cfg_out,                                                          \
		.make_thread = udc_rts_make_thread_##n,                                            \
		.speed_idx = DT_ENUM_IDX(DT_DRV_INST(n), maximum_speed),                           \
		.irq_enable_func = udc_rts_irq_enable_func_##n,                                    \
		.irq_disable_func = udc_rts_irq_disable_func_##n,                                  \
		.usb.u2sie_sys_base = DT_REG_ADDR_BY_IDX(DT_NODELABEL(usb), 0),                    \
		.usb.u2sie_ep_base = DT_REG_ADDR_BY_IDX(DT_NODELABEL(usb), 1),                     \
		.usb.u2mc_ep_base = DT_REG_ADDR_BY_IDX(DT_NODELABEL(usb), 2),                      \
		.usb.usb2_ana_base = DT_REG_ADDR_BY_IDX(DT_NODELABEL(usb), 3),                     \
	};                                                                                         \
                                                                                                   \
	static struct udc_rts_data udc_priv_##n = {};                                              \
                                                                                                   \
	static struct udc_data udc_data_##n = {                                                    \
		.mutex = Z_MUTEX_INITIALIZER(udc_data_##n.mutex),                                  \
		.priv = &udc_priv_##n,                                                             \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, udc_rts_driver_preinit, NULL, &udc_data_##n, &udc_rts_config_##n, \
			      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &udc_rts_api);

DT_INST_FOREACH_STATUS_OKAY(UDC_RTS5816_DEVICE_DEFINE)
