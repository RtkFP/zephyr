# Copyright (c) 2021 Google Inc.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_BBRAM_SHELL bbram_shell.c)

zephyr_library_sources_ifdef(CONFIG_USERSPACE bbram_handlers.c)
zephyr_library_sources_ifdef(CONFIG_BBRAM_NPCX bbram_npcx.c)
zephyr_library_sources_ifdef(CONFIG_BBRAM_NPCX_EMUL bbram_npcx_emul.c)
zephyr_library_include_directories_ifdef(CONFIG_BBRAM_NPCX .)
zephyr_library_sources_ifdef(CONFIG_BBRAM_IT8XXX2 bbram_it8xxx2.c)
zephyr_library_sources_ifdef(CONFIG_BBRAM_IT8XXX2_EMUL bbram_it8xxx2_emul.c)
zephyr_library_include_directories_ifdef(CONFIG_BBRAM_IT8XXX2 .)
zephyr_library_sources_ifdef(CONFIG_BBRAM_EMUL bbram_emul.c)
zephyr_library_sources_ifdef(CONFIG_BBRAM_MICROCHIP_MCP7940N bbram_microchip_mcp7940n.c)
zephyr_library_sources_ifdef(CONFIG_BBRAM_MICROCHIP_MCP7940N_EMUL bbram_microchip_mcp7940n_emul.c)
zephyr_library_sources_ifdef(CONFIG_BBRAM_XEC bbram_xec.c)
zephyr_library_sources_ifdef(CONFIG_BBRAM_STM32 bbram_stm32.c)
zephyr_library_sources_ifdef(CONFIG_BBRAM_RTS5912 bbram_rts5912.c)
