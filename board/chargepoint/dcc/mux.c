/*
 * mux.c
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <common.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/hardware.h>
#include <asm/arch/mux.h>
#include <asm/io.h>
#include <i2c.h>
#include "board.h"

static struct module_pin_mux uart0_pin_mux[] = {
	{OFFSET(uart0_rxd), (MODE(0) | PULLUP_EN | RXACTIVE)},	/* UART0_RXD */
	{OFFSET(uart0_txd), (MODE(0) | PULLUDEN)},		/* UART0_TXD */
	{-1},
};

static struct module_pin_mux uart1_pin_mux[] = {
	{OFFSET(uart1_rxd), (MODE(0) | PULLUP_EN | RXACTIVE)},	/* UART1_RXD */
	{OFFSET(uart1_txd), (MODE(0) | PULLUDEN)},		/* UART1_TXD */
	{-1},
};

static struct module_pin_mux uart2_pin_mux[] = {
	{OFFSET(spi0_sclk), (MODE(1) | PULLUP_EN | RXACTIVE)},	/* UART2_RXD */
	{OFFSET(spi0_d0), (MODE(1) | PULLUDEN)},		/* UART2_TXD */
	{-1},
};

static struct module_pin_mux uart3_pin_mux[] = {
	{OFFSET(spi0_cs1), (MODE(1) | PULLUP_EN | RXACTIVE)},	/* UART3_RXD */
	{OFFSET(ecap0_in_pwm0_out), (MODE(1) | PULLUDEN)},	/* UART3_TXD */
	{-1},
};

static struct module_pin_mux uart4_pin_mux[] = {
	{OFFSET(gpmc_wait0), (MODE(6) | PULLUP_EN | RXACTIVE)},	/* UART4_RXD */
	{OFFSET(gpmc_wpn), (MODE(6) | PULLUDEN)},		/* UART4_TXD */
	{-1},
};

static struct module_pin_mux uart5_pin_mux[] = {
	{OFFSET(lcd_data9), (MODE(4) | PULLUP_EN | RXACTIVE)},	/* UART5_RXD */
	{OFFSET(lcd_data8), (MODE(4) | PULLUDEN)},		/* UART5_TXD */
	{-1},
};

static struct module_pin_mux mmc0_pin_mux[] = {
	{OFFSET(mmc0_dat3), (MODE(0) | RXACTIVE | PULLUP_EN)},  /* MMC0_DAT3 */
	{OFFSET(mmc0_dat2), (MODE(0) | RXACTIVE | PULLUP_EN)},  /* MMC0_DAT2 */
	{OFFSET(mmc0_dat1), (MODE(0) | RXACTIVE | PULLUP_EN)},  /* MMC0_DAT1 */
	{OFFSET(mmc0_dat0), (MODE(0) | RXACTIVE | PULLUP_EN)},  /* MMC0_DAT0 */
	{OFFSET(mmc0_clk), (MODE(0) | RXACTIVE | PULLUP_EN)},   /* MMC0_CLK */
	{OFFSET(mmc0_cmd), (MODE(0) | RXACTIVE | PULLUP_EN)},   /* MMC0_CMD */
	{OFFSET(ecap0_in_pwm0_out),
	        (MODE(7) | RXACTIVE | PULLUP_EN)}, /* GPIO0_7 */
	{-1},
};

static struct module_pin_mux mmc1_pin_mux_dccv2[] = {
	{OFFSET(gpmc_ad3), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT3 */
	{OFFSET(gpmc_ad2), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT2 */
	{OFFSET(gpmc_ad1), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT1 */
	{OFFSET(gpmc_ad0), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT0 */
	{OFFSET(gpmc_csn1), (MODE(2) | RXACTIVE | PULLUP_EN)},	/* MMC1_CLK */
	{OFFSET(gpmc_csn2), (MODE(2) | RXACTIVE | PULLUP_EN)},	/* MMC1_CMD */
	{OFFSET(gpmc_wen), (MODE(7) | RXACTIVE | PULLUP_EN)},   /* GPIO2_4 */
	{-1},
};

static struct module_pin_mux mmc1_pin_mux[] = {
	{OFFSET(gpmc_ad7), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT7 */
	{OFFSET(gpmc_ad6), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT6 */
	{OFFSET(gpmc_ad5), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT5 */
	{OFFSET(gpmc_ad4), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT4 */
	{OFFSET(gpmc_ad3), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT3 */
	{OFFSET(gpmc_ad2), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT2 */
	{OFFSET(gpmc_ad1), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT1 */
	{OFFSET(gpmc_ad0), (MODE(1) | RXACTIVE | PULLUP_EN)},	/* MMC1_DAT0 */
	{OFFSET(gpmc_csn1), (MODE(2) | RXACTIVE | PULLUP_EN)},	/* MMC1_CLK */
	{OFFSET(gpmc_csn2), (MODE(2) | RXACTIVE | PULLUP_EN)},	/* MMC1_CMD */
	{-1},
};

static struct module_pin_mux i2c0_pin_mux[] = {
	{OFFSET(i2c0_sda), (MODE(0) | RXACTIVE |
			PULLUDEN | SLEWCTRL)}, /* I2C_DATA */
	{OFFSET(i2c0_scl), (MODE(0) | RXACTIVE |
			PULLUDEN | SLEWCTRL)}, /* I2C_SCLK */
	{-1},
};

static struct module_pin_mux usb0_pin_mux[] = {
	{OFFSET(usb0_dm), (MODE(0) | RXACTIVE | PULLUDDIS)},
	{OFFSET(usb0_dp), (MODE(0) | RXACTIVE | PULLUDDIS)},
	{OFFSET(usb0_ce), (MODE(0) | RXACTIVE | PULLUDDIS)},
	{OFFSET(usb0_id), (MODE(0) | RXACTIVE | PULLUDDIS)},
	{OFFSET(usb0_vbus), (MODE(0) | RXACTIVE | PULLUDDIS)},
	{OFFSET(usb0_drvvbus), (MODE(0))},
	{-1},
};

static struct module_pin_mux usb1_pin_mux[] = {
	{OFFSET(usb1_dm), (MODE(0) | RXACTIVE | PULLUDDIS)},
	{OFFSET(usb1_dp), (MODE(0) | RXACTIVE | PULLUDDIS)},
	{OFFSET(usb1_ce), (MODE(0) | RXACTIVE | PULLUDDIS)},
	{OFFSET(usb1_id), (MODE(0) | RXACTIVE | PULLUDDIS)},
	{OFFSET(usb1_vbus), (MODE(0) | RXACTIVE | PULLUDDIS)},
	{OFFSET(usb1_drvvbus), (MODE(0))},
	{-1},
};


static struct module_pin_mux spi0_pin_mux[] = {
	{OFFSET(spi0_sclk), (MODE(0) | RXACTIVE | PULLUDEN)},	/* SPI0_SCLK */
	{OFFSET(spi0_d0), (MODE(0) | RXACTIVE |
			PULLUDEN | PULLUP_EN)},			/* SPI0_D0 */
	{OFFSET(spi0_d1), (MODE(0) | RXACTIVE | PULLUDEN)},	/* SPI0_D1 */
	{OFFSET(spi0_cs0), (MODE(0) | RXACTIVE |
			PULLUDEN | PULLUP_EN)},		/* SPI0_CS0 */
	{OFFSET(spi0_cs1), (MODE(0) | RXACTIVE |
			PULLUDEN | PULLUP_EN)},			/* SPI0_CS1 */
	{-1},
};

static struct module_pin_mux gpio_pin_mux[] = {
	{OFFSET(mii1_txd2), (MODE(7))}, /* mii1.txd2.gpio0_17 */
	{OFFSET(mii1_txd3), (MODE(7))}, /* mii1.txd2.gpio0_16 */
	{OFFSET(gpmc_ad10), (MODE(7) | PULLUP_EN)}, /* GPIO0_26 */
	{OFFSET(gpmc_ad11), (MODE(7))},	/* GPIO0_27 */
	{OFFSET(gpmc_wait0), (MODE(7))}, /* GPIO0_30 */
	{OFFSET(gpmc_ad12), (MODE(7) | PULLUP_EN)}, /* gpmc_ad12.gpio1_12 */
	{OFFSET(gpmc_ad13), (MODE(7))}, /* gpmc_ad13.gpio1_13 */
	{OFFSET(gpmc_ad14), (MODE(7))}, /* gpmc_ad14.gpio1_14 */
	{OFFSET(gpmc_ad15), (MODE(7) | PULLUP_EN)}, /* apmc_ad15.gpio1_15 */
	{OFFSET(gpmc_a2), (MODE(7))}, /* gpmc_a2.gpio1_18 */
	{OFFSET(gpmc_a3), (MODE(7))}, /* gpmc_a3.gpio1_19 */
	{OFFSET(gpmc_a8), (MODE(7))}, /* gpmc_a8.gpio1_24 */
	{OFFSET(gpmc_clk), (MODE(7) | PULLUP_EN)}, /* GPIO2_1 */
	{OFFSET(gpmc_be0n_cle), (MODE(7) | PULLDOWN_EN)}, /* GPIO2_5 */
	{OFFSET(lcd_vsync), (MODE(7) | PULLUP_EN)}, /* GPIO2_22 */
	{OFFSET(lcd_hsync), (MODE(7) | PULLUP_EN)}, /* GPIO2_23 */
	{OFFSET(mii1_rxdv), (MODE(7))}, /* GPIO3_4 */
	{OFFSET(mii1_txclk), (MODE(7))}, /* GPIO3_9 */
	{OFFSET(mii1_rxclk), (MODE(7))}, /* GPIO3_10 */
	{OFFSET(mcasp0_ahclkr), (MODE(7))}, /* GPIO3_17 */
	{OFFSET(mcasp0_aclkr), (MODE(7))}, /* GPIO3_18 */
	{OFFSET(mcasp0_fsr), (MODE(7))}, /* GPIO3_19 */
	{OFFSET(mcasp0_axr1), (MODE(7))}, /* GPIO3_20 */
	{OFFSET(mcasp0_ahclkx), (MODE(7))}, /* GPIO3_21 */
	{-1},
};

static struct module_pin_mux gpio_pin_mux_dccv2[] = {
	{OFFSET(lcd_pclk), (MODE(7))},	/* GPIO2_24 */
	{OFFSET(lcd_ac_bias_en), (MODE(7))},	/* GPIO2_25 */
	{OFFSET(mmc0_dat1), (MODE(7))},	/* GPIO2_28 */
	{OFFSET(gpmc_be1n), (MODE(7))},	/* GPIO1_28 */
	{-1},
};

static struct module_pin_mux gpio_pin_mux_dccv3[] = {
	{OFFSET(lcd_pclk), (MODE(7) | PULLUP_EN)}, /* GPIO2_24 */
	{OFFSET(lcd_ac_bias_en), (MODE(7) | PULLUP_EN)}, /* GPIO2_25 */
	{-1},
};

static struct module_pin_mux gpio_pin_mux_dccv4[] = {
	{OFFSET(lcd_pclk), (MODE(7) | PULLUP_EN)}, /* GPIO2_24 */
	{OFFSET(lcd_ac_bias_en), (MODE(7) | PULLUP_EN)}, /* GPIO2_25 */
	{-1},
};

static struct module_pin_mux rmii1_pin_mux[] = {
	{OFFSET(mii1_txen), MODE(1)},                   /* RMII1_TXEN */
	{OFFSET(mii1_txd1), MODE(1)},                   /* RMII1_TD1 */
	{OFFSET(mii1_txd0), MODE(1)},                   /* RMII1_TD0 */
	{OFFSET(mii1_rxd1), MODE(1) | RXACTIVE},        /* RMII1_RD1 */
	{OFFSET(mii1_rxd0), MODE(1) | RXACTIVE},        /* RMII1_RD0 */
	{OFFSET(mii1_crs), MODE(1) | RXACTIVE},         /* RMII1_CRS_DV */
	{OFFSET(mii1_rxerr), MODE(1) | RXACTIVE},       /* RMII1_RXERR */
	{OFFSET(rmii1_refclk), MODE(0) | RXACTIVE},     /* RMII1_refclk */
	{-1},
};

static struct module_pin_mux mdio_pin_mux[] = {
	{OFFSET(mdio_data), MODE(0) | RXACTIVE | PULLUP_EN},/* MDIO_DATA */
	{OFFSET(mdio_clk), MODE(0) | PULLUP_EN},        /* MDIO_CLK */
	{-1},
};

static struct module_pin_mux rmii2_pin_mux[] = {
	{OFFSET(gpmc_a0), MODE(3)},                     /* RMII2_TXEN */
	{OFFSET(gpmc_a4), MODE(3)},                     /* RMII2_TD1 */
	{OFFSET(gpmc_a5), MODE(3)},                     /* RMII2_TD0 */
	{OFFSET(gpmc_a10), MODE(3) | RXACTIVE},         /* RMII2_RD1 */
	{OFFSET(gpmc_a11), MODE(3) | RXACTIVE},         /* RMII2_RD0 */
	{OFFSET(gpmc_a9), MODE(3) | RXACTIVE},          /* RMII2_CRS_DV */
	{OFFSET(gpmc_wpn), MODE(3) | RXACTIVE},         /* RMII2_RXERR */
	{OFFSET(mii1_col), MODE(1) | RXACTIVE},         /* RMII2_refclk */
	{-1},
};

static struct module_pin_mux board_id_pin_mux[] = {
	{OFFSET(gpmc_advn_ale), (MODE(7))},  /* gpmc_advn_ale.gpio2_2 */
	{OFFSET(gpmc_oen_ren), (MODE(7))},  /* gpmc_oen_ren.gpio2_3 */
	{-1},
};

void enable_gpio_pin_mux(void)
{
	configure_module_pin_mux(gpio_pin_mux);
}

void enable_gpio_pin_mux_dccv2(void)
{
	configure_module_pin_mux(gpio_pin_mux_dccv2);
}

void enable_gpio_pin_mux_dccv3(void)
{
	configure_module_pin_mux(gpio_pin_mux_dccv3);
}

void enable_gpio_pin_mux_dccv4(void)
{
	configure_module_pin_mux(gpio_pin_mux_dccv4);
}

void enable_uart0_pin_mux(void)
{
	configure_module_pin_mux(uart0_pin_mux);
}

void enable_uart1_pin_mux(void)
{
	configure_module_pin_mux(uart1_pin_mux);
}

void enable_uart2_pin_mux(void)
{
	configure_module_pin_mux(uart2_pin_mux);
}

void enable_uart3_pin_mux(void)
{
	configure_module_pin_mux(uart3_pin_mux);
}

void enable_uart4_pin_mux(void)
{
	configure_module_pin_mux(uart4_pin_mux);
}

void enable_uart5_pin_mux(void)
{
	configure_module_pin_mux(uart5_pin_mux);
}

void enable_i2c0_pin_mux(void)
{
	configure_module_pin_mux(i2c0_pin_mux);
}

void enable_boardid_mux(void)
{
	configure_module_pin_mux(board_id_pin_mux);
}

void enable_board_pin_mux(struct am335x_baseboard_id *header,
			  enum am335x_dcc_board board_type)
{
	enable_gpio_pin_mux();
	configure_module_pin_mux(mdio_pin_mux);

	/* Do board-specific muxes. */
	if (board_type == DCC_BOARD_REV2) {
		enable_gpio_pin_mux_dccv2();
		configure_module_pin_mux(mmc1_pin_mux_dccv2);
		configure_module_pin_mux(rmii1_pin_mux);
	} else {
		if (board_type == DCC_BOARD_REV3) {
			enable_gpio_pin_mux_dccv3();
		} else {
			enable_gpio_pin_mux_dccv4();
		}
		configure_module_pin_mux(mmc0_pin_mux);
		configure_module_pin_mux(mmc1_pin_mux);
		configure_module_pin_mux(usb0_pin_mux);
		configure_module_pin_mux(usb1_pin_mux);
		configure_module_pin_mux(rmii1_pin_mux);
		configure_module_pin_mux(rmii2_pin_mux);
	}
}
