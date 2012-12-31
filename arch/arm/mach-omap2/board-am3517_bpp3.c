/*
 * Copyright (C) 2011 Ilya Yanok, Emcraft Systems
 *
 * Modified from mach-omap2/board-am3517_mt_ventoux.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/plat-ram.h>
#include <linux/mmc/host.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/spi/ads7846.h>
#include <linux/spi/spi.h>
#include <linux/can/platform/ti_hecc.h>
#include <linux/uio_driver.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <plat/common.h>
#include <video/omapdss.h>
#include <video/omap-panel-generic-dpi.h>
#include <plat/usb.h>

#include "am35xx.h"

#include "am35xx-emac.h"
#include "mux.h"
#include "control.h"
#include "hsmmc.h"
#include "common-board-devices.h"
#include "common.h"

#define MCX_MDIO_FREQUENCY	(1000000)

static struct mtd_partition bpp3_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "SPL",
		.offset		= 0,
		.size		= 4 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 8 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot Env1",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x180000 */
		.size		= 2 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "U-Boot Env2",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x1C0000 */
		.size		= 2 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x200000 */
		.size		= 48 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x800000 */
		.size		= MTDPART_SIZ_FULL,
	},
};

#define LED_PWR_PIN		53
#define PANEL_PWR_PIN		138
#define LCD_PANEL_PON_PIN	139

static int bpp3_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(LCD_PANEL_PON_PIN, 1);
	gpio_set_value(PANEL_PWR_PIN, 0);
	gpio_set_value(LED_PWR_PIN, 1);

	return 0;
}

static void bpp3_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(LED_PWR_PIN, 0);
	gpio_set_value(PANEL_PWR_PIN, 1);
	gpio_set_value(LCD_PANEL_PON_PIN, 0);
}

static struct panel_generic_dpi_data lcd_panel = {
	.name			= "ortustech_com43h4m10xtc",
	.platform_enable	= bpp3_panel_enable_lcd,
	.platform_disable	= bpp3_panel_disable_lcd,
};

static struct omap_dss_device bpp3_lcd_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "lcd",
	.driver_name		= "generic_dpi_panel",
	.data			= &lcd_panel,
	.phy.dpi.data_lines	= 24,
};

static struct omap_dss_device *bpp3_dss_devices[] = {
	&bpp3_lcd_device,
};

static struct omap_dss_board_info bpp3_dss_data = {
	.num_devices	= ARRAY_SIZE(bpp3_dss_devices),
	.devices	= bpp3_dss_devices,
	.default_device	= &bpp3_lcd_device,
};

/*
 * use fake regulator for vdds_dsi as we can't find this pin inside
 * AM3517 datasheet.
 */
static struct regulator_consumer_supply bpp3_vdds_dsi_supply[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi.0"),
};

static struct regulator_init_data bpp3_vdds_dsi = {
	.constraints		= {
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.always_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(bpp3_vdds_dsi_supply),
	.consumer_supplies	= bpp3_vdds_dsi_supply,
};

static struct fixed_voltage_config bpp3_display = {
	.supply_name		= "display",
	.microvolts		= 1800000,
	.gpio			= -EINVAL,
	.enabled_at_boot	= 1,
	.init_data		= &bpp3_vdds_dsi,
};

static struct platform_device bpp3_display_device = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev = {
		.platform_data	= &bpp3_display,
	},
};

static struct gpio bpp3_dss_gpios[] __initdata = {
	{
		.gpio = LED_PWR_PIN,
		.flags = GPIOF_OUT_INIT_HIGH,
		.label = "backlight_power",
	},
	{
		.gpio = LCD_PANEL_PON_PIN,
		.flags = GPIOF_OUT_INIT_HIGH,
		.label = "lcd_disable_vdd",
	},
	{
		.gpio = PANEL_PWR_PIN,
		.flags = GPIOF_OUT_INIT_LOW,
		.label = "lcd_vdd_disable_pon",
	},
};

static void __init bpp3_display_init(void)
{
	int r;

	omap_mux_init_gpio(PANEL_PWR_PIN, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(LCD_PANEL_PON_PIN, OMAP_PIN_OUTPUT);
	omap_mux_init_gpio(LED_PWR_PIN, OMAP_PIN_OUTPUT);

	r = gpio_request_array(bpp3_dss_gpios, ARRAY_SIZE(bpp3_dss_gpios));
	if (r) {
		pr_err("failed to get DSS control GPIOs\n");
		return;
	}

	r = omap_display_init(&bpp3_dss_data);
	if (r) {
		pr_err("Failed to register DSS device\n");
		gpio_free_array(bpp3_dss_gpios, ARRAY_SIZE(bpp3_dss_gpios));
	}
}

static struct gpio_keys_button bpp3_gpio_buttons[] = {
	{
		.code		= KEY_UP,
		.gpio		= 28,
		.desc		= "up",
		.wakeup		= 1,
		.active_low	= 1,
	},
	{
		.code		= KEY_RIGHT,
		.gpio		= 137,
		.desc		= "right",
		.wakeup		= 1,
		.active_low	= 1,
	},
	{
		.code		= KEY_DOWN,
		.gpio		= 152,
		.desc		= "down",
		.wakeup		= 1,
		.active_low	= 1,
	},
	{
		.code		= KEY_LEFT,
		.gpio		= 29,
		.desc		= "left",
		.wakeup		= 1,
		.active_low	= 1,
	},
	{
		.code		= KEY_SPACE,
		.gpio		= 52,
		.desc		= "middle",
		.wakeup		= 1,
		.active_low	= 1,
	},
	{
		.code		= KEY_ESC,
		.gpio		= 26,
		.desc		= "esc",
		.wakeup		= 1,
		.active_low	= 1,
	},
	{
		.code		= KEY_ENTER,
		.gpio		= 127,
		.desc		= "enter",
		.wakeup		= 1,
		.active_low	= 1,
	},
};

static struct gpio_keys_platform_data bpp3_gpio_key_info = {
	.buttons	= bpp3_gpio_buttons,
	.nbuttons	= ARRAY_SIZE(bpp3_gpio_buttons),
};

static struct platform_device bpp3_keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data  = &bpp3_gpio_key_info,
	},
};

static void set_gpio_key_mux(void)
{
	int n;

	for (n = 0; n < ARRAY_SIZE(bpp3_gpio_buttons); n++)
		omap_mux_init_gpio(bpp3_gpio_buttons[n].gpio, OMAP_PIN_INPUT);
}

/* TPS65023 specific initialization */
/* VDCDC1 -> VDD_CORE */
static struct regulator_consumer_supply am3517_vdcdc1_supplies[] = {
	{
		.supply = "vdd_core",
	},
};

/* VDCDC2 -> VDDSHV */
static struct regulator_consumer_supply am3517_vdcdc2_supplies[] = {
	{
		.supply = "vddshv",
	},
};

/*
 * VDCDC2 |-> VDDS
 *	  |-> VDDS_SRAM_CORE_BG
 *	  |-> VDDS_SRAM_MPU
 */
static struct regulator_consumer_supply am3517_vdcdc3_supplies[] = {
	{
		.supply = "vdds",
	},
	{
		.supply = "vdds_sram_core_bg",
	},
	{
		.supply = "vdds_sram_mpu",
	},
};

/*
 * LDO1 |-> VDDA1P8V_USBPHY
 *	|-> VDDA_DAC
 */
static struct regulator_consumer_supply am3517_ldo1_supplies[] = {
	{
		.supply = "vdda1p8v_usbphy",
	},
	{
		.supply = "vdda_dac",
	},
};

/* LDO2 -> VDDA3P3V_USBPHY */
static struct regulator_consumer_supply am3517_ldo2_supplies[] = {
	{
		.supply = "vdda3p3v_usbphy",
	},
};

static struct regulator_init_data bpp3_regulator_data[] = {
	/* DCDC1 */
	{
		.constraints = {
			.min_uV = 1200000,
			.max_uV = 1200000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = true,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(am3517_vdcdc1_supplies),
		.consumer_supplies = am3517_vdcdc1_supplies,
	},
	/* DCDC2 */
	{
		.constraints = {
			.min_uV = 3300000,
			.max_uV = 3300000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = true,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(am3517_vdcdc2_supplies),
		.consumer_supplies = am3517_vdcdc2_supplies,
	},
	/* DCDC3 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = true,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(am3517_vdcdc3_supplies),
		.consumer_supplies = am3517_vdcdc3_supplies,
	},
	/* LDO1 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = false,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(am3517_ldo1_supplies),
		.consumer_supplies = am3517_ldo1_supplies,
	},
	/* LDO2 */
	{
		.constraints = {
			.min_uV = 3300000,
			.max_uV = 3300000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = false,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(am3517_ldo2_supplies),
		.consumer_supplies = am3517_ldo2_supplies,
	},
};

static struct i2c_board_info __initdata bpp3_i2c1_devices[] = {
	{
		I2C_BOARD_INFO("tps65023", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &bpp3_regulator_data[0],
	},
	{
		I2C_BOARD_INFO("24c02", 0x50),
	},
};

static struct i2c_board_info __initdata bpp3_i2c3_devices[] = {
	{
		I2C_BOARD_INFO("ds1307", 0x68),
	},
};

static void __init bpp3_i2c_init(void)
{
	omap_register_i2c_bus(1, 400, bpp3_i2c1_devices,
			ARRAY_SIZE(bpp3_i2c1_devices));
	omap_register_i2c_bus(3, 400, bpp3_i2c3_devices,
			ARRAY_SIZE(bpp3_i2c3_devices));
}


#define USB_PHY1_RESET		25

static const struct usbhs_omap_board_data usbhs_bdata __initconst = {

	.port_mode[0] = OMAP_EHCI_PORT_MODE_PHY,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_USBHS_PORT_MODE_UNUSED,

	.phy_reset  = true,
	.reset_gpio_port[0]  = USB_PHY1_RESET,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL,
};

#define SD_CARD_CD		126

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= SD_CARD_CD,
		.gpio_wp	= -EINVAL,
	},
	{}      /* Terminator */
};

static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

static struct resource am3517_hecc_resources[] = {
	{
		.start	= AM35XX_IPSS_HECC_BASE,
		.end	= AM35XX_IPSS_HECC_BASE + 0x3FFF,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= 24 + OMAP_INTC_START,
		.flags	= IORESOURCE_IRQ,
	},
};
static struct ti_hecc_platform_data am3517_hecc_pdata = {
	.scc_hecc_offset	= AM35XX_HECC_SCC_HECC_OFFSET,
	.scc_ram_offset		= AM35XX_HECC_SCC_RAM_OFFSET,
	.hecc_ram_offset	= AM35XX_HECC_RAM_OFFSET,
	.mbx_offset		= AM35XX_HECC_MBOX_OFFSET,
	.int_line		= AM35XX_HECC_INT_LINE,
	.version		= AM35XX_HECC_VERSION,
};

static struct platform_device am3517_hecc_device = {
	.name		= "ti_hecc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(am3517_hecc_resources),
	.resource	= am3517_hecc_resources,
	.dev		= {
		.platform_data = &am3517_hecc_pdata,
	},
};

static void am3517_init_hecc(void)
{
	omap_mux_init_signal("hecc1_txd", OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_signal("hecc1_rxd", OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP);

	platform_device_register(&am3517_hecc_device);
}

static struct platform_device *bpp3_devices[] __initdata = {
	&bpp3_display_device,
	&bpp3_keys_gpio,
};

/* uart1 -> console, uart2 -> mk2, no flow control */
static struct omap_device_pad uart1_pads[] __initdata = {
	{
		.name	= "uart1_tx.uart1_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart1_rx.uart1_rx",
		.flags	= OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
		.idle	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
};

static struct omap_device_pad uart2_pads[] __initdata = {
	{
		.name	= "uart2_tx.uart2_tx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart2_rx.uart2_rx",
		.flags	= OMAP_DEVICE_PAD_REMUX | OMAP_DEVICE_PAD_WAKEUP,
		.enable	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
		.idle	= OMAP_PIN_INPUT_PULLUP | OMAP_MUX_MODE0,
	},
};

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_PERIPHERAL,
	.set_phy_power		= am35x_musb_phy_power,
	.clear_irq		= am35x_musb_clear_irq,
	.set_mode		= am35x_set_mode,
	.reset			= am35x_musb_reset,
};

static __init void am3517_musb_init(void)
{
	u32 devconf2;

	/* Set up USB clock/mode in the DEVCONF2 register. */
	devconf2 = omap_ctrl_readl(AM35XX_CONTROL_DEVCONF2);

	/* USB2.0 PHY reference clock is 13 MHz */
	devconf2 &= ~(CONF2_REFFREQ | CONF2_OTGMODE | CONF2_PHY_GPIOMODE);
	devconf2 |=  CONF2_REFFREQ_13MHZ | CONF2_SESENDEN | CONF2_VBDTCTEN
		        | CONF2_DATPOL;

	omap_ctrl_writel(devconf2, AM35XX_CONTROL_DEVCONF2);

	usb_musb_init(&musb_board_data);
}

static inline void bpp3_serial_init(void)
{
	struct omap_board_data bdata;

	bdata.flags = 0;

	bdata.id = 0;
	bdata.pads = uart1_pads;
	bdata.pads_cnt = ARRAY_SIZE(uart1_pads);
	omap_serial_init_port(&bdata, NULL);

	bdata.id = 1;
	bdata.pads = uart2_pads;
	bdata.pads_cnt = ARRAY_SIZE(uart2_pads);
	omap_serial_init_port(&bdata, NULL);
}

static struct gpio bpp3_gpio_export[] = {
	{
		.gpio = 42,
		.flags = GPIOF_OUT_INIT_LOW,
		.label = "relay",
	},
	{
		.gpio = 54,
		.flags = GPIOF_OUT_INIT_LOW,
		.label = "buzzer",
	},
	{
		.gpio = 99,
		.flags = GPIOF_OUT_INIT_HIGH,
		.label = "power_down",
	},
	{
		.gpio = 104,
		.flags = GPIOF_OUT_INIT_HIGH,
		.label = "power",
	},
	{
		.gpio = 153,
		.flags = GPIOF_OUT_INIT_LOW,
		.label = "vebus_standby",
	},
	{
		.gpio = 155,
		.flags = GPIOF_OUT_INIT_HIGH,
		.label = "mk2_reset",
	},
};

static void __init bpp3_export_gpio(void)
{
	int n, r;

	for (n = 0; n < ARRAY_SIZE(bpp3_gpio_export); n++)
		omap_mux_init_gpio(bpp3_gpio_export[n].gpio, OMAP_PIN_OUTPUT);

	r = gpio_request_array(bpp3_gpio_export, ARRAY_SIZE(bpp3_gpio_export));
	if (r) {
		printk(KERN_WARNING "failed to request export gpio\n");
		return;
	}

	for (n = 0; n < ARRAY_SIZE(bpp3_gpio_export); n++)
		if (gpio_export(bpp3_gpio_export[n].gpio, 0))
			printk(KERN_WARNING "failed to export gpio %d\n",
					bpp3_gpio_export[n].gpio);
}

static void __init bpp3_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_ZCN);
	set_gpio_key_mux();
	bpp3_i2c_init();

	/*
	 * Keep the left button high. If this is not done the board will
	 * reset, since the F2 button powers the board and loses power
	 * when F1 and F2 are pressed.
	 */
	gpio_request(26, "F1");
	omap_mux_init_gpio(26, OMAP_PIN_INPUT_PULLUP);
	gpio_free(26);

	platform_add_devices(bpp3_devices, ARRAY_SIZE(bpp3_devices));
	bpp3_serial_init();
	omap_sdrc_init(NULL, NULL);

	bpp3_display_init();

	/* CAN */
	am3517_init_hecc();

	/* Configure EHCI ports */
	omap_mux_init_gpio(USB_PHY1_RESET, OMAP_PIN_OUTPUT);
	usbhs_init(&usbhs_bdata);

	/* NAND */
	omap_nand_flash_init(NAND_BUSWIDTH_16, bpp3_nand_partitions,
			     ARRAY_SIZE(bpp3_nand_partitions));

	/* Ethernet */
	am35xx_emac_init(AM35XX_DEFAULT_MDIO_FREQUENCY, 1);

	/* MMC init */
	omap_mux_init_signal("sdmmc1_dat4.gpio_126", OMAP_PIN_INPUT);
	omap_mux_init_signal("cam_strobe.safe_mode", OMAP_PIN_OFF_NONE);
	omap_hsmmc_init(mmc);

	/* USB Peripheral */
	am3517_musb_init();

	bpp3_export_gpio();
}

MACHINE_START(AM3517_BPP3, "Victron BPP3")
	.atag_offset	= 0x100,
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= am35xx_init_early,
	.init_irq	= omap3_init_irq,
	.handle_irq	= omap3_intc_handle_irq,
	.init_machine	= bpp3_init,
	.init_late	= am35xx_init_late,
	.timer		= &omap3_timer,
	.restart	= omap_prcm_restart,
MACHINE_END
