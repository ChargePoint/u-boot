/*
 * board.c
 *
 * Board functions for TI AM335X based boards
 *
 * Copyright (C) 2011, Texas Instruments, Incorporated - http://www.ti.com/
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <errno.h>
#include <spl.h>
#include <asm/arch/cpu.h>
#include <asm/arch/hardware.h>
#include <asm/arch/omap.h>
#include <asm/arch/ddr_defs.h>
#include <asm/arch/clock.h>
#include <asm/arch/gpio.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mem.h>
#include <asm/io.h>
#include <asm/emif.h>
#include <asm/gpio.h>
#include <i2c.h>
#include <miiphy.h>
#include <cpsw.h>
#include <power/tps65217.h>
#include <power/tps65910.h>
#include <environment.h>
#include <watchdog.h>
#include <environment.h>
#include "board.h"

DECLARE_GLOBAL_DATA_PTR;

#if defined(CONFIG_SPL_BUILD) || \
    (defined(CONFIG_DRIVER_TI_CPSW) && !defined(CONFIG_DM_ETH))
	static struct ctrl_dev *cdev = (struct ctrl_dev *)CTRL_DEVICE_BASE;
#endif

static enum am335x_dcc_board board_type = DCC_BOARD_NONE;

#define GPIO_BOARD_ID0 66 /* gpio2_2 */
#define GPIO_BOARD_ID1 67 /* gpio2_3 */
int get_board_revision(void)
{
	int ret;
	int revision;
	int id0 = 0;
	int id1 = 0;

	ret = gpio_request(GPIO_BOARD_ID0, "board-id0");
	if (ret && ret != -EBUSY) {
		printf("gpio: requesting pin %u failed\n", GPIO_BOARD_ID0);
	} else {
		gpio_direction_input(GPIO_BOARD_ID0);
		id0 = gpio_get_value(GPIO_BOARD_ID0);
		gpio_free(GPIO_BOARD_ID0);
	}

	ret = gpio_request(GPIO_BOARD_ID1, "board-id1");
	if (ret && ret != -EBUSY) {
		printf("gpio: requesting pin %u failed\n", GPIO_BOARD_ID1);
	} else {
		gpio_direction_input(GPIO_BOARD_ID1);
		id1 = gpio_get_value(GPIO_BOARD_ID1);
		gpio_free(GPIO_BOARD_ID1);
	}

	revision = (id1 << 1) | id0;
	printf("board rev=%d (id1=%d id0=%d)\n", revision, id1, id0);
	return revision;
}

/*
 * Read header information from EEPROM into global structure.
 */
static int read_eeprom(struct am335x_baseboard_id *header)
{
	int board_rev = DCC_BOARD_NONE;

	/*
	 * Check if eeprom is available. With DCC V3/V4
	 * should see eeprom on I2C bus
	 */
	if (i2c_probe(CONFIG_SYS_I2C_EEPROM_ADDR)) {
		printf("DCC REV2 board detected.\n");
		board_type = DCC_BOARD_REV2;
	} else {
		board_rev = get_board_revision();
		if (!board_rev) {
			printf("DCC REV3 board detected.\n");
			board_type = DCC_BOARD_REV3;
		} else {
			printf("DCC REV4 board detected\n");
			board_type = DCC_BOARD_REV4;
		}
	}

	/* This information need to be read from eeprom */
	memset(header,0,sizeof(struct am335x_baseboard_id));
	header->magic = 0xEE3355AA;
	strcpy(header->name,"A335X_SK");
	strcpy(header->version, "2.0");   //"2.0" for DCC
	strcpy(header->serial,"36154P160337");
	strcpy(header->config,"SKU#01");
	memset(header->mac_addr[0],0xFF,6);
	memset(header->mac_addr[1],0xFF,6);
	memset(header->mac_addr[2],0xFF,6);

	return 0;
}

#ifndef CONFIG_SKIP_LOWLEVEL_INIT
static const struct ddr_data ddr3_dcc_data = {
	.datardsratio0 = MT41K128M16JT125E_RD_DQS,
	.datawdsratio0 = MT41K128M16JT125E_WR_DQS,
	.datafwsratio0 = MT41K128M16JT125E_PHY_FIFO_WE,
	.datawrsratio0 = MT41K128M16JT125E_PHY_WR_DATA,
};

static const struct cmd_control ddr3_dcc_cmd_ctrl_data = {
	.cmd0csratio = MT41K128M16JT125E_RATIO,
	.cmd0iclkout = MT41K128M16JT125E_INVERT_CLKOUT,

	.cmd1csratio = MT41K128M16JT125E_RATIO,
	.cmd1iclkout = MT41K128M16JT125E_INVERT_CLKOUT,

	.cmd2csratio = MT41K128M16JT125E_RATIO,
	.cmd2iclkout = MT41K128M16JT125E_INVERT_CLKOUT,
};

static struct emif_regs ddr3_dcc_emif_reg_data = {
	.sdram_config = MT41K128M16JT125E_EMIF_SDCFG,
	.ref_ctrl = MT41K128M16JT125E_EMIF_SDREF,
	.sdram_tim1 = MT41K128M16JT125E_EMIF_TIM1,
	.sdram_tim2 = MT41K128M16JT125E_EMIF_TIM2,
	.sdram_tim3 = MT41K128M16JT125E_EMIF_TIM3,
	.zq_config = MT41K128M16JT125E_ZQ_CFG,
	.emif_ddr_phy_ctlr_1 = MT41K128M16JT125E_EMIF_READ_LATENCY,
};

static const struct ddr_data ddr3_dccv3_data = {
	.datardsratio0 = MT41K256M16TW107_RD_DQS,
	.datawdsratio0 = MT41K256M16TW107_WR_DQS,
	.datafwsratio0 = MT41K256M16TW107_PHY_FIFO_WE,
	.datawrsratio0 = MT41K256M16TW107_PHY_WR_DATA,
};

static const struct cmd_control ddr3_dccv3_cmd_ctrl_data = {
	.cmd0csratio = MT41K256M16TW107_RATIO,
	.cmd0iclkout = MT41K256M16TW107_INVERT_CLKOUT,

	.cmd1csratio = MT41K256M16TW107_RATIO,
	.cmd1iclkout = MT41K256M16TW107_INVERT_CLKOUT,

	.cmd2csratio = MT41K256M16TW107_RATIO,
	.cmd2iclkout = MT41K256M16TW107_INVERT_CLKOUT,
};

static struct emif_regs ddr3_dccv3_emif_reg_data = {
	.sdram_config = MT41K256M16TW107_EMIF_SDCFG,
	.ref_ctrl = MT41K256M16TW107_EMIF_SDREF,
	.sdram_tim1 = MT41K256M16TW107_EMIF_TIM1,
	.sdram_tim2 = MT41K256M16TW107_EMIF_TIM2,
	.sdram_tim3 = MT41K256M16TW107_EMIF_TIM3,
	.zq_config = MT41K256M16TW107_ZQ_CFG,
	.emif_ddr_phy_ctlr_1 = MT41K256M16TW107_EMIF_READ_LATENCY,
};

#ifdef CONFIG_SPL_OS_BOOT
int spl_start_uboot(void)
{
	/* break into full u-boot on 'c' */
	if (serial_tstc() && serial_getc() == 'c')
		return 1;

#ifdef CONFIG_SPL_ENV_SUPPORT
	env_init();
	env_relocate_spec();
	if (getenv_yesno("boot_os") != 1)
		return 1;
#endif

	return 0;
}
#endif

#define OSC	(V_OSCK/1000000)
const struct dpll_params dpll_ddr = {
		266, OSC-1, 1, -1, -1, -1, -1};
const struct dpll_params dpll_ddr_evm_sk = {
		303, OSC-1, 1, -1, -1, -1, -1};
const struct dpll_params dpll_ddr_dcc = {
		400, OSC-1, 1, -1, -1, -1, -1};

void am33xx_spl_board_init(void)
{
	struct am335x_baseboard_id header;
	int mpu_vdd;

	enable_boardid_mux();
	enable_i2c0_pin_mux();
	i2c_init(CONFIG_SYS_OMAP24_I2C_SPEED, CONFIG_SYS_OMAP24_I2C_SLAVE);

	if (read_eeprom(&header) < 0)
		puts("Could not get board ID.\n");

	/* Set CORE Frequencies to OPP100*/
	do_setup_dpll(&dpll_core_regs, &dpll_core_opp100);
	dpll_mpu_opp100.m = MPUPLL_M_500;
	/* Set MPU Frequency to what we detected now that voltages are set*/
	do_setup_dpll(&dpll_mpu_regs, &dpll_mpu_opp100);
}

const struct dpll_params *get_dpll_ddr_params(void)
{
	return &dpll_ddr_dcc;  /* DCC use DDR at 400 Mhz */
}

void set_uart_mux_conf(void)
{
#if CONFIG_CONS_INDEX == 1
	enable_uart0_pin_mux();
#elif CONFIG_CONS_INDEX == 2
	enable_uart1_pin_mux();
#elif CONFIG_CONS_INDEX == 3
	enable_uart2_pin_mux();
#elif CONFIG_CONS_INDEX == 4
	enable_uart3_pin_mux();
#elif CONFIG_CONS_INDEX == 5
	enable_uart4_pin_mux();
#elif CONFIG_CONS_INDEX == 6
	enable_uart5_pin_mux();
#endif
}

void set_mux_conf_regs(void)
{
	__maybe_unused struct am335x_baseboard_id header;

	/*
	 * Ref Sitara TRM 'sma2' Register Field Descriptions
	 *
	 * rmii2_crs_dv_mode_sel (U16) requires another level of mux select
	 * due to conflict with MMC2_DAT7 pin.
	 *
	 *   0: Select MMC2_DAT7 on GPMC_A9 pin in MODE3.
	 *   1: Select RMII2_CRS_DV on GPMC_A9 pin in MODE3.
	 *
	 * This needs to be done in the bootloader because the devicetree
	 * doesn't have a resource for this and the initialization is
	 * not a guaranteed order and so it needs to be set to ensure
	 * the mmc interface can be detected as expected.
	 */
#ifndef SMA2_REG_ADDR
#define SMA2_REG_ADDR 0x44e11320
#endif
	writel(0x1, SMA2_REG_ADDR); /* select rmii2 */

	enable_boardid_mux();
	enable_i2c0_pin_mux();
	i2c_init(CONFIG_SYS_OMAP24_I2C_SPEED, CONFIG_SYS_OMAP24_I2C_SLAVE);

	if (read_eeprom(&header) < 0)
		puts("Could not get board ID.\n");
	enable_board_pin_mux(&header, board_type);
}

const struct ctrl_ioregs ioregs_dcc = {
	.cm0ioctl = MT41K128M16JT125E_IOCTRL_VALUE,
	.cm1ioctl = MT41K128M16JT125E_IOCTRL_VALUE,
	.cm2ioctl = MT41K128M16JT125E_IOCTRL_VALUE,
	.dt0ioctl = MT41K128M16JT125E_IOCTRL_VALUE,
	.dt1ioctl = MT41K128M16JT125E_IOCTRL_VALUE,
};

const struct ctrl_ioregs ioregs_dccv3 = {
	.cm0ioctl = MT41K256M16TW107_IOCTRL_VALUE,
	.cm1ioctl = MT41K256M16TW107_IOCTRL_VALUE,
	.cm2ioctl = MT41K256M16TW107_IOCTRL_VALUE,
	.dt0ioctl = MT41K256M16TW107_IOCTRL_VALUE,
	.dt1ioctl = MT41K256M16TW107_IOCTRL_VALUE,
};

void sdram_init(void)
{
	if (board_type == DCC_BOARD_REV2) {
		config_ddr(400, &ioregs_dcc, &ddr3_dcc_data,
			&ddr3_dcc_cmd_ctrl_data, &ddr3_dcc_emif_reg_data, 0);
	} else {
		config_ddr(400, &ioregs_dccv3, &ddr3_dccv3_data,
			&ddr3_dccv3_cmd_ctrl_data,
			&ddr3_dccv3_emif_reg_data, 0);
	}
}
#endif

/*
 * Basic board specific setup.  Pinmux has been handled already.
 */
int board_init(void)
{
	u32 sys_reboot;

	sys_reboot = readl(PRM_RSTST);
	if (sys_reboot & (1 << 9))
		puts("Reset Source: IcePick reset has occurred.\n");

	if (sys_reboot & (1 << 5))
		puts("Reset Source: Global external warm reset has occurred.\n");

	if (sys_reboot & (1 << 4))
		puts("Reset Source: watchdog reset has occurred.\n");

	if (sys_reboot & (1 << 1))
		puts("Reset Source: Global warm SW reset has occurred.\n");

	if (sys_reboot & (1 << 0))
		puts("Reset Source: Power-on reset has occurred.\n");
#if defined(CONFIG_HW_WATCHDOG)
	hw_watchdog_init();
#endif

	gd->bd->bi_boot_params = CONFIG_SYS_SDRAM_BASE + 0x100;
#if defined(CONFIG_NOR) || defined(CONFIG_NAND)
	gpmc_init();
#endif

	if (board_type == DCC_BOARD_REV2) {
/* Drive MCP2515 Reset GPIO(GPIO02_28) for DCCV2 to output low to force reset */
#define GPIO_MCP2515_RESETV2 92
		gpio_request(GPIO_MCP2515_RESETV2, "mcp2515_nrst");
		gpio_direction_output(GPIO_MCP2515_RESETV2, 0);
		gpio_free(GPIO_MCP2515_RESETV2);
#define GPIO_ETH_SW_RESET 60
		gpio_request(GPIO_ETH_SW_RESET, "eth_reset");
		gpio_direction_output(GPIO_ETH_SW_RESET, 0);
		gpio_free(GPIO_ETH_SW_RESET);
	} else {
/* Drive MCP2515 Reset GPIO(GPIO01_15) to output low to force reset */
#define GPIO_MCP2515_RESET 47
		gpio_request(GPIO_MCP2515_RESET, "mcp2515_nrst");
		gpio_direction_output(GPIO_MCP2515_RESET, 0);
		gpio_free(GPIO_MCP2515_RESET);
/* CHAD1/CHAD2n GPIO2_5 */
#define GPIO_CHAD1_CHAD2_N 69
		gpio_request(GPIO_CHAD1_CHAD2_N, "chad1_chad2n");
		gpio_direction_output(GPIO_CHAD1_CHAD2_N, 1);
		gpio_free(GPIO_CHAD1_CHAD2_N);
/* CHAdeMO_1_Charge GPIO2_22 */
#define GPIO_CHADM1_CHRG 86
		gpio_request(GPIO_CHADM1_CHRG, "CHAdeMO_1_Charge");
		gpio_direction_output(GPIO_CHADM1_CHRG, 1);
		gpio_free(GPIO_CHADM1_CHRG);
/* CHAdeMO_1_Proximity GPIO0_26 */
#define GPIO_CHADM1_PROX 26
		gpio_request(GPIO_CHADM1_PROX, "CHAdeMO_1_Proximity");
		gpio_direction_output(GPIO_CHADM1_PROX, 1);
		gpio_free(GPIO_CHADM1_PROX);
/* CHAdeMO_2_Charge GPIO2_23 */
#define GPIO_CHADM2_CHRG 87
		gpio_request(GPIO_CHADM2_CHRG, "CHAdeMO_2_Charge");
		gpio_direction_output(GPIO_CHADM2_CHRG, 1);
		gpio_free(GPIO_CHADM2_CHRG);
/* Output_Pilot_1_PWM_n GPIO1_12 */
#define GPIO_OUT_PILOT1_PWM 44
		gpio_request(GPIO_OUT_PILOT1_PWM, "Output_Pilot_1_PWM_n");
		gpio_direction_output(GPIO_OUT_PILOT1_PWM, 0);
		gpio_free(GPIO_OUT_PILOT1_PWM);
/* Output_Tesla_PWM_n GPIO2_1 */
#define GPIO_TESLA_PWM 65
		gpio_request(GPIO_TESLA_PWM, "Output_Tesla_PWM_n");
		gpio_direction_output(GPIO_TESLA_PWM, 0);
		gpio_free(GPIO_TESLA_PWM);
/* I2C0_Reset_n GPIO3_21 */
#define GPIO_I2C0_RESET 117
		gpio_request(GPIO_I2C0_RESET, "GPIO_I2C0_RESET");
		gpio_direction_output(GPIO_I2C0_RESET, 0);
		gpio_free(GPIO_I2C0_RESET);
/* Cable_I2C_reset GPIO0_30 */
#define GPIO_CABLE_I2C_RESET 30
		gpio_request(GPIO_CABLE_I2C_RESET, "Cable_I2C_Reset_n");
		gpio_direction_output(GPIO_CABLE_I2C_RESET, 0);
		gpio_free(GPIO_CABLE_I2C_RESET);
/* CHAdeMO_12V_Enable  GPIO3_17*/
#define GPIO_CHAD_12VEN 113
		gpio_request(GPIO_CHAD_12VEN, "CHAdeMO_12V_Enable");
		gpio_direction_output(GPIO_CHAD_12VEN, 0);
		gpio_free(GPIO_CHAD_12VEN);
/* CHAdeMO_L_P12V_ON    GPIO3_9*/
#define GPIO_CHAD_L_12VEN 105
		gpio_request(GPIO_CHAD_L_12VEN, "CHAdeMO_L_P12V_ON");
		gpio_direction_output(GPIO_CHAD_L_12VEN, 0);
		gpio_free(GPIO_CHAD_L_12VEN);
/* CHAdeMO_L_P12V_STAT    GPIO1_24*/
#define GPIO_CHAD_L_12VSTAT 56
		gpio_request(GPIO_CHAD_L_12VSTAT, "CHAdeMO_L_P12V_STAT");
		gpio_direction_input(GPIO_CHAD_L_12VSTAT);
		gpio_free(GPIO_CHAD_L_12VSTAT);
/* CHAdeMO_1_Sequence_1 GPIO1_13 */
#define GPIO_CHAD1_SEQ1 45
		gpio_request(GPIO_CHAD1_SEQ1, "CHAdeMO_1_Sequence_1");
		gpio_direction_output(GPIO_CHAD1_SEQ1, 0);
		gpio_free(GPIO_CHAD1_SEQ1);
/* CHAdeMO_1_Sequence_2 GPIO1_14 */
#define GPIO_CHAD1_SEQ2 46
		gpio_request(GPIO_CHAD1_SEQ2, "CHAdeMO_1_Sequence_2");
		gpio_direction_output(GPIO_CHAD1_SEQ2, 0);
		gpio_free(GPIO_CHAD1_SEQ2);
/* CHAdeMO_1_Seq_Det_n GPIO0_17 */
#define GPIO_CHAD1_SEQDET 17
		gpio_request(GPIO_CHAD1_SEQDET, "CHAdeMO_1_Seq_Det_n");
		gpio_direction_input(GPIO_CHAD1_SEQDET);
		gpio_free(GPIO_CHAD1_SEQDET);
/* CHAdeMO_1_L_EN GPIO1_19 */
#define GPIO_CHAD1_LEN 51
		gpio_request(GPIO_CHAD1_LEN, "CHAdeMO_1_L_EN");
		gpio_direction_output(GPIO_CHAD1_LEN, 0);
		gpio_free(GPIO_CHAD1_LEN);
/* CHAdeMO_2_Sequence_1 GPIO3_19 */
#define GPIO_CHAD2_SEQ1 115
		gpio_request(GPIO_CHAD2_SEQ1, "CHAdeMO_2_Sequence_1");
		gpio_direction_output(GPIO_CHAD2_SEQ1, 0);
		gpio_free(GPIO_CHAD2_SEQ1);
/* CHAdeMO_2_Sequence_2 GPIO3_20 */
#define GPIO_CHAD2_SEQ2 116
		gpio_request(GPIO_CHAD2_SEQ2, "CHAdeMO_2_Sequence_2");
		gpio_direction_output(GPIO_CHAD2_SEQ2, 0);
		gpio_free(GPIO_CHAD2_SEQ2);
/* CHAdeMO_2_Seq_Det_n GPIO0_16 */
#define GPIO_CHAD2_SEQDET 16
		gpio_request(GPIO_CHAD2_SEQDET, "CHAdeMO_2_Seq_Det_n");
		gpio_direction_input(GPIO_CHAD2_SEQDET);
		gpio_free(GPIO_CHAD2_SEQDET);
/* CHAdeMO_2_L_EN GPIO1_18 */
#define GPIO_CHAD2_LEN 50
		gpio_request(GPIO_CHAD2_LEN, "CHAdeMO_2_L_EN");
		gpio_direction_output(GPIO_CHAD2_LEN, 0);
		gpio_free(GPIO_CHAD2_LEN);
	}

/* Drive PLC1/PLC2 Reset GPIO(GPIO2_24, GPIO2_25) to output low */
#define GPIO_PLC1_RESET 88
	gpio_request(GPIO_PLC1_RESET, "plc1_rst");
	gpio_direction_output(GPIO_PLC1_RESET, 0);
	gpio_free(GPIO_PLC1_RESET);

#define GPIO_PLC2_RESET 89
	gpio_request(GPIO_PLC2_RESET, "plc2_rst");
	gpio_direction_output(GPIO_PLC2_RESET, 0);
	gpio_free(GPIO_PLC2_RESET);

	/* ensure plcs are in reset for at least 500ms */
	mdelay(500);

	return 0;
}

#ifdef CONFIG_BOARD_LATE_INIT
int board_late_init(void)
{
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	char safe_string[HDR_NAME_LEN + 1];
	__maybe_unused struct am335x_baseboard_id header;

	if (read_eeprom(&header) < 0)
		puts("Could not get board ID.\n");

	/* Now set variables based on the header. */
	strncpy(safe_string, (char *)header.name, sizeof(header.name));
	safe_string[sizeof(header.name)] = 0;
	setenv("board_name", safe_string);

	strncpy(safe_string, (char *)header.version, sizeof(header.version));
	safe_string[sizeof(header.version)] = 0;
	setenv("board_rev", safe_string);
#endif

	/* disable auto boot delay (AUTOBOOT_KEYED for ctrl-C protection) */
	setenv("bootdelay", "0");

	return 0;
}
#endif

#ifndef CONFIG_DM_ETH

#if (defined(CONFIG_DRIVER_TI_CPSW) && !defined(CONFIG_SPL_BUILD)) || \
	(defined(CONFIG_SPL_ETH_SUPPORT) && defined(CONFIG_SPL_BUILD))
static void cpsw_control(int enabled)
{
	/* VTP can be added here */

	return;
}

static struct cpsw_slave_data cpsw_slaves[] = {
	{
		.slave_reg_ofs	= 0x208,
		.sliver_reg_ofs	= 0xd80,
		.phy_addr	= 0,
	},
	{
		.slave_reg_ofs	= 0x308,
		.sliver_reg_ofs	= 0xdc0,
		.phy_addr	= 1,
	},
};

static struct cpsw_platform_data cpsw_data = {
	.mdio_base		= CPSW_MDIO_BASE,
	.cpsw_base		= CPSW_BASE,
	.mdio_div		= 0xff,
	.channels		= 8,
	.cpdma_reg_ofs		= 0x800,
	.slaves			= 1,
	.slave_data		= cpsw_slaves,
	.ale_reg_ofs		= 0xd00,
	.ale_entries		= 1024,
	.host_port_reg_ofs	= 0x108,
	.hw_stats_reg_ofs	= 0x900,
	.bd_ram_ofs		= 0x2000,
	.mac_control		= (1 << 5),
	.control		= cpsw_control,
	.host_port_num		= 0,
	.version		= CPSW_CTRL_VERSION_2,
};
#endif

/*
 * This function will:
 * Read the eFuse for MAC addresses, and set ethaddr/eth1addr/usbnet_devaddr
 * in the environment
 * Perform fixups to the PHY present on certain boards.  We only need this
 * function in:
 * - SPL with either CPSW or USB ethernet support
 * - Full U-Boot, with either CPSW or USB ethernet
 * Build in only these cases to avoid warnings about unused variables
 * when we build an SPL that has neither option but full U-Boot will.
 */
#if ((defined(CONFIG_SPL_ETH_SUPPORT) || defined(CONFIG_SPL_USBETH_SUPPORT)) \
		&& defined(CONFIG_SPL_BUILD)) || \
	((defined(CONFIG_DRIVER_TI_CPSW) || \
	  defined(CONFIG_USB_ETHER) && defined(CONFIG_USB_MUSB_GADGET)) && \
	 !defined(CONFIG_SPL_BUILD))
int board_eth_init(bd_t *bis)
{
	int rv, n = 0;
	uint8_t mac_addr[6];
	uint32_t mac_hi, mac_lo;
	__maybe_unused struct am335x_baseboard_id header;

	/* try reading mac address from efuse */
	mac_lo = readl(&cdev->macid0l);
	mac_hi = readl(&cdev->macid0h);
	mac_addr[0] = mac_hi & 0xFF;
	mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
	mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
	mac_addr[4] = mac_lo & 0xFF;
	mac_addr[5] = (mac_lo & 0xFF00) >> 8;

#if (defined(CONFIG_DRIVER_TI_CPSW) && !defined(CONFIG_SPL_BUILD)) || \
	(defined(CONFIG_SPL_ETH_SUPPORT) && defined(CONFIG_SPL_BUILD))
	if (!getenv("ethaddr")) {
		printf("<ethaddr> not set. Validating first E-fuse MAC\n");

		if (is_valid_ethaddr(mac_addr))
			eth_setenv_enetaddr("ethaddr", mac_addr);
	}

#ifdef CONFIG_DRIVER_TI_CPSW

	mac_lo = readl(&cdev->macid1l);
	mac_hi = readl(&cdev->macid1h);
	mac_addr[0] = mac_hi & 0xFF;
	mac_addr[1] = (mac_hi & 0xFF00) >> 8;
	mac_addr[2] = (mac_hi & 0xFF0000) >> 16;
	mac_addr[3] = (mac_hi & 0xFF000000) >> 24;
	mac_addr[4] = mac_lo & 0xFF;
	mac_addr[5] = (mac_lo & 0xFF00) >> 8;

	if (!getenv("eth1addr")) {
		if (is_valid_ethaddr(mac_addr))
			eth_setenv_enetaddr("eth1addr", mac_addr);
	}

	if (board_type == DCC_BOARD_REV2) {
		/* Rev2 board has only one RMII phy connected to GMII1 port */
		writel((GMII1_SEL_RMII | RMII1_IO_CLK_EN), &cdev->miisel);

		cpsw_slaves[0].phy_if =	PHY_INTERFACE_MODE_RMII;
		cpsw_slaves[0].phy_addr = 0;
	} else {
		/* Rev3/4 board has 2 RMII LAN phy connected */
		writel((RMII_MODE_ENABLE | RMII_CHIPCKL_ENABLE), &cdev->miisel);

		cpsw_slaves[0].phy_if =	PHY_INTERFACE_MODE_RMII;
		cpsw_slaves[0].phy_addr = 0;
		cpsw_slaves[1].phy_if =	PHY_INTERFACE_MODE_RMII;
		cpsw_slaves[1].phy_addr = 1;
	}

	rv = cpsw_register(&cpsw_data);
	if (rv < 0)
		printf("Error %d registering CPSW switch\n", rv);
	else
		n += rv;
#endif
#endif
#if defined(CONFIG_USB_ETHER) && \
	(!defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_USBETH_SUPPORT))
	if (is_valid_ethaddr(mac_addr))
		eth_setenv_enetaddr("usbnet_devaddr", mac_addr);

	rv = usb_eth_initialize(bis);
	if (rv < 0)
		printf("Error %d registering USB_ETHER\n", rv);
	else
		n += rv;
#endif
	return n;
}
#endif

#endif /* CONFIG_DM_ETH */
