.. rts5816_demoboard:

rts5816_demoboard
#################

Overview
********

rts5816 is a high-performance MCU based on Realtek Real-M300 ARMv8-M architecture with Cortex-M33 instruction set compatible.

.. figure:: img/rts5816_demoboard.jpg
   :width: 400px
   :align: center
   :alt: rts5816_demoboard

Hardware
********

- 240MHz single-core 32-bit CPU with I-cache 32KB, D-cache 16KB
- 48KB boot ROM
- 256KB on-chip SRAM
- 1KB PUFrt OTP
- 2MB Flash
- USB2.0 full speed/high speed device
- Up tp 24 GPIO
- 2 x UART
- 2 x Timer
- 1 x I2C, support both master and slave mode
- 2 x SPI, one can support slave mode
- Watchdog
- PWM
- DMA
- Temperature sensor
- Cryptographic hardware acceleration (RNG, ECC, RSA, SHA, AES)
- Fingerprint mataching hardware acceleration (MAC, popcount, convolution, etc.)
- SWD for debug

Supported Features
==================

For now, the following board hardware features are supported by implemented drivers:

+-------------------+------------+----------------------+
| Interface         | Controller | Driver/Component     |
+===================+============+======================+
| CLOCK   	    | on-chip    | clock_control        |
+-------------------+------------+----------------------+
| FLASH   	    | on-chip    | flash                |
+-------------------+------------+----------------------+
| GPIO    	    | on-chip    | gpio                 |
+-------------------+------------+----------------------+
| SPI(M/S) 	    | on-chip    | spi                  |
+-------------------+------------+----------------------+
| UART	            | on-chip    | serial               |
+-------------------+------------+----------------------+
| USB               | on-chip    | usb                  |
+-------------------+------------+----------------------+
| WDT               | on-chip    | watchdog             |
+-------------------+------------+----------------------+
| PUF               | on-chip    | puf                  |
+-------------------+------------+----------------------+
| Temperature sensor| on-chip    | sensor               |
+-------------------+------------+----------------------+

Programming and Debugging
*************************

Flashing
========

Here is an example for building the hello_world application.

.. zephyr-app-commands::
   :zephyr-app: samples/hello_world
   :board: rts5816_demoboard
   :goals: build flash

The USB device to burn image can be selected by options ``--pid``,
if not provided, the default vendor_id 0bda and product_id 5816 will be used.

.. code-block:: console

	$ west flash --pid=0bda:5816
