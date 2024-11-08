/*
 * Copyright (c) 2024 Realtek Semiconductor, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SPI_SPI_RTS5816_H_
#define ZEPHYR_DRIVERS_SPI_SPI_RTS5816_H_

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/spi.h>

#include "spi_context.h"
#include <stdint.h>

typedef void (*spi_rts5816_irq_config_t)(void);

/* Private structures */
struct spi_rts5816_config {
	uint32_t ctrl_regs; /* ctrl regs base */
	uint32_t dw_regs;   /* dw regs base */
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	spi_rts5816_irq_config_t irq_config_func;
	bool serial_target;
	bool interrupt_enable;
	uint8_t max_xfer_size;
#ifdef CONFIG_PINCTRL
	const struct pinctrl_dev_config *pcfg;
#endif
};

struct spi_rts5816_data {
	struct spi_context ctx;
	uint32_t clock_frequency;
	uint8_t dfs;             /* dfs in bytes: 1,2 or 4 */
	size_t xfer_len;         /* transceived length */
	uint8_t *aligned_buffer; /* alloc aligned buffer pointer */
};
#define DW_SPI_CTRLR0_DFS_16(__bpw) ((__bpw) - 1)
#define DW_SPI_CTRLR0_DFS_32(__bpw) (((__bpw) - 1) << 16)

#define RTS5816_SPI_TIME_OUT_MS 1000

#define SPI_WS_TO_DFS(__bpw) (((__bpw) & ~0x38) ? (((__bpw) / 8) + 1) : ((__bpw) / 8))

#define SPI_RTS_CLK_DIVIDER(clock_freq, ssi_clk_hz) ((clock_freq / ssi_clk_hz) & 0xFFFF)

#endif
