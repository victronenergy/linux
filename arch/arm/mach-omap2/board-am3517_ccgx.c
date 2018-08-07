/*
 * Copyright (C) 2011 Ilya Yanok, Emcraft Systems
 *
 * Modified from mach-omap2/board-am3517_mt_ventoux.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/plat-ram.h>
#include <linux/mmc/host.h>
#include <linux/proc_fs.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/machine.h>
#include <linux/sysfs.h>
#include <linux/spi/ads7846.h>
#include <linux/spi/spi.h>
#include <linux/can/platform/ti_hecc.h>
#include <linux/uio_driver.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <plat/common.h>
#include <plat/dmtimer.h>
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

struct gptimer_pwm_dev {
	int id;
	struct omap_dm_timer *timer;
	u32 input_freq;
	u32 tldr;
	u32 tmar;
	u32 num_settings;
	u32 current_val;
	u32 set;
};

static struct mtd_partition ccgx_nand_partitions[] = {
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

static struct panel_generic_dpi_data lcd_panel = {
	.name			= "ortustech_com43h4m10xtc",
};

static struct omap_dss_device ccgx_lcd_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "lcd",
	.driver_name		= "generic_dpi_panel",
	.data			= &lcd_panel,
	.phy.dpi.data_lines	= 24,
};

static struct omap_dss_device *ccgx_dss_devices[] = {
	&ccgx_lcd_device,
};

static struct omap_dss_board_info ccgx_dss_data = {
	.num_devices	= ARRAY_SIZE(ccgx_dss_devices),
	.devices	= ccgx_dss_devices,
	.default_device	= &ccgx_lcd_device,
};

/*
 * use fake regulator for vdds_dsi as we can't find this pin inside
 * AM3517 datasheet.
 */
static struct regulator_consumer_supply ccgx_vdds_dsi_supply[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi.0"),
};

static struct regulator_init_data ccgx_vdds_dsi = {
	.constraints		= {
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.always_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(ccgx_vdds_dsi_supply),
	.consumer_supplies	= ccgx_vdds_dsi_supply,
};

static struct fixed_voltage_config ccgx_display = {
	.supply_name		= "display",
	.microvolts		= 1800000,
	.gpio			= -EINVAL,
	.enabled_at_boot	= 1,
	.init_data		= &ccgx_vdds_dsi,
};

static struct platform_device ccgx_display_device = {
	.name		= "reg-fixed-voltage",
	.id		= 0,
	.dev = {
		.platform_data	= &ccgx_display,
	},
};

static void __init ccgx_display_init(void)
{
	int r;

	r = omap_display_init(&ccgx_dss_data);
	if (r) {
		pr_err("Failed to register DSS device\n");
	}
}

static struct gptimer_pwm_dev gptimer9 = {
	.id = 9,
};

static void __init gptimer9_pwm_set_frequency(struct gptimer_pwm_dev *pd)
{
	int frequency = 1024;

	if (frequency > (pd->input_freq / 2))
		frequency = pd->input_freq / 2;

	pd->tldr = 0xFFFFFFFF - (pd->input_freq / frequency - 1);
	omap_dm_timer_set_load(pd->timer, 1, pd->tldr);
	pd->num_settings = 0xFFFFFFFE - pd->tldr;
}

static int gptimer9_pwm_set_duty_cycle(u32 duty_cycle)
{
	u32 new_tmar;

	if (gptimer9.set != 1) {
		printk("%s: pwm_init fail or not executed.\n", __FUNCTION__);
		return -EINVAL;
	}

	if (duty_cycle > 100)
		return -EINVAL;

	if (duty_cycle == 0) {
		if (gptimer9.current_val != 0) {
			omap_dm_timer_stop(gptimer9.timer);
			gptimer9.current_val = 0;
		}
		return 0;
	}

	new_tmar = duty_cycle * gptimer9.num_settings / 100;

	if (new_tmar < 1)
		new_tmar = 1;
	else if (new_tmar > gptimer9.num_settings)
		new_tmar = gptimer9.num_settings;

	gptimer9.tmar = gptimer9.tldr + new_tmar;
	omap_dm_timer_set_match(gptimer9.timer, 1, gptimer9.tmar);

	if (gptimer9.current_val == 0)
		omap_dm_timer_start(gptimer9.timer);

	gptimer9.current_val = duty_cycle;

	return 0;
}


static void gptimer9_pwm_timer_cleanup(void)
{
	printk(KERN_INFO "pwm: gptimer9_pwm_timer_cleanup\n");

	if (gptimer9.timer) {
		omap_dm_timer_free(gptimer9.timer);
		gptimer9.timer = NULL;
		gptimer9.set = 0;
	}
}

static int __init gptimer9_pwm_timer_init(void)
{
	struct clk *fclk;

	printk(KERN_INFO "pwm: omap_dm_timer_request_specific\n");
	gptimer9.timer = omap_dm_timer_request_specific(gptimer9.id);

	if (!gptimer9.timer)
		goto timer_init_fail;

	printk(KERN_INFO "pwm: omap_dm_timer_set_pwm\n");
	omap_dm_timer_set_pwm(gptimer9.timer,
				0,	/* ~SCPWM low when off */
				1,	/* PT pulse toggle modulation */
				OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);

	if (omap_dm_timer_set_source(gptimer9.timer, OMAP_TIMER_SRC_SYS_CLK))
		goto timer_init_fail;

	/* set the clock frequency */
	fclk = omap_dm_timer_get_fclk(gptimer9.timer);
	gptimer9.input_freq = clk_get_rate(fclk);
	gptimer9_pwm_set_frequency(&gptimer9);
	gptimer9.set = 1;

	return 0;

timer_init_fail:
	gptimer9_pwm_timer_cleanup();
	return -1;
}

int __init gptimer9_pwm_init(void)
{
	if (gptimer9_pwm_timer_init()) {
		gptimer9_pwm_timer_cleanup();
		return -1;
	}

	return 0;
}

static int backlight_set_status(struct backlight_device *bl)
{
	int level;

	if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
		bl->props.power == FB_BLANK_UNBLANK)
		level = bl->props.brightness;
	else
		level = 0;

	return gptimer9_pwm_set_duty_cycle(level);
}

static int backlight_get_brightness(struct backlight_device *bl)
{
	if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
			bl->props.power == FB_BLANK_UNBLANK)
		return bl->props.brightness;

	return 0;
}

static const struct backlight_ops backlight_ops = {
	.get_brightness	= backlight_get_brightness,
	.update_status	= backlight_set_status,
};

static struct backlight_properties __initdata backlight_props = {
	.type		= BACKLIGHT_RAW,
	.max_brightness	= 100,
	.brightness	= 100,
	.power		= FB_BLANK_UNBLANK,
	.fb_blank	= FB_BLANK_UNBLANK,
};

static void __init backlight_init(void)
{
	struct backlight_device *backlight;

	backlight = backlight_device_register("bpp3-bl", NULL, NULL,
					&backlight_ops, &backlight_props);
	if (IS_ERR(backlight)) {
		printk(KERN_ERR "backlight error %ld\n", PTR_ERR(backlight));
		return;
	}

	if (!gptimer9_pwm_init()) {
		gptimer9_pwm_set_duty_cycle(backlight_props.brightness);
		omap_mux_init_signal("gpmc_ncs2", OMAP_MUX_MODE2);
	}
}

static struct gpio_keys_button ccgx_gpio_buttons[] = {
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

static struct gpio_keys_platform_data ccgx_gpio_key_info = {
	.buttons	= ccgx_gpio_buttons,
	.rep		= 1,
	.nbuttons	= ARRAY_SIZE(ccgx_gpio_buttons),
};

static struct platform_device ccgx_keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data  = &ccgx_gpio_key_info,
	},
};

static void set_gpio_key_mux(void)
{
	int n;

	for (n = 0; n < ARRAY_SIZE(ccgx_gpio_buttons); n++)
		omap_mux_init_gpio(ccgx_gpio_buttons[n].gpio, OMAP_PIN_INPUT);
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

static struct regulator_init_data ccgx_regulator_data[] = {
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

static struct i2c_board_info __initdata ccgx_i2c1_devices[] = {
	{
		I2C_BOARD_INFO("tps65023", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &ccgx_regulator_data[0],
	},
	{
		I2C_BOARD_INFO("24c02", 0x50),
	},
};

static struct i2c_board_info __initdata ccgx_i2c3_devices[] = {
	{
		I2C_BOARD_INFO("ds1307", 0x68),
	},
};

static void __init ccgx_i2c_init(void)
{
	omap_register_i2c_bus(1, 400, ccgx_i2c1_devices,
			ARRAY_SIZE(ccgx_i2c1_devices));
	omap_register_i2c_bus(3, 400, ccgx_i2c3_devices,
			ARRAY_SIZE(ccgx_i2c3_devices));
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

#define CANBUS_ENABLE_GPIO		117
static void hecc_phy_control(int on)
{
	int r;

	if (on) {
		printk("turning CAN driver on\n");
		r = gpio_request(CANBUS_ENABLE_GPIO, "can_enable");
		if (r)
			goto trouble;
		if (gpio_direction_output(CANBUS_ENABLE_GPIO, 1))
			goto trouble;
	} else {
		printk("turning CAN driver off\n");
		if (gpio_direction_output(CANBUS_ENABLE_GPIO, 0))
			goto trouble;
		gpio_free(CANBUS_ENABLE_GPIO);
	}

	return;

trouble:
	printk(KERN_ERR "failed to control CAN_power (%d)\n", on);
}

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
	.transceiver_switch	= hecc_phy_control,
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
	omap_mux_init_gpio(CANBUS_ENABLE_GPIO, OMAP_PIN_OUTPUT);

	platform_device_register(&am3517_hecc_device);
}

static struct platform_device *ccgx_devices[] __initdata = {
	&ccgx_display_device,
	&ccgx_keys_gpio,
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

static struct omap_device_pad uart3_pads[] __initdata = {
	{
		.name	= "uart3_tx_irtx.uart3_tx_irtx",
		.enable	= OMAP_PIN_OUTPUT | OMAP_MUX_MODE0,
	},
	{
		.name	= "uart3_rx_irrx.uart3_rx_irrx",
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

/* Galvanication isolation */
#define MK2_ISOLATOR_POWER_GPIO		116

static inline void ccgx_serial_init(void)
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
	gpio_request(MK2_ISOLATOR_POWER_GPIO, "mk2_isolation");
	omap_mux_init_gpio(MK2_ISOLATOR_POWER_GPIO, OMAP_PIN_OUTPUT);
	gpio_direction_output(MK2_ISOLATOR_POWER_GPIO, 1);
	omap_serial_init_port(&bdata, NULL);

	bdata.id = 2;
	bdata.pads = uart3_pads;
	bdata.pads_cnt = ARRAY_SIZE(uart3_pads);
	omap_serial_init_port(&bdata, NULL);
}

static struct gpio ccgx_gpio_export[] = {
	{
		.gpio = 35,
		.flags = GPIOF_OUT_INIT_LOW,
		.label = "buzzer",
	},
	{
		.gpio = 94,
		.flags = GPIOF_IN,
		.label = "hw_revR821",
	},
	{
		.gpio = 96,
		.flags = GPIOF_IN,
		.label = "hw_revR819",
	},
	{
		.gpio = 98,
		.flags = GPIOF_IN,
		.label = "hw_revR820",
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
		.flags = GPIOF_OUT_INIT_HIGH,
		.label = "vebus_standby",
	},
	{
		.gpio = 155,
		.flags = GPIOF_OUT_INIT_HIGH,
		.label = "mk2_reset",
	},
	{
		.gpio = 182,
		.flags = GPIOF_OUT_INIT_LOW,
		.label = "relay",
	},
};

static void __init ccgx_export_gpio(void)
{
	int n;

	for (n = 0; n < ARRAY_SIZE(ccgx_gpio_export); n++) {
		if (ccgx_gpio_export[n].flags == GPIOF_IN)
			/*
			 * This can generate a "Multiple gpio paths (2) for
			 * gpio##" info message. The OMAP am35xx_zcn_subset
			 * has multiple definitions for the same pin. The mux
			 * mode for the pins is set separately using
			 * "omap_mux_init_signal" from ccgx_init
			 */
			omap_mux_init_gpio(ccgx_gpio_export[n].gpio,
						OMAP_PIN_INPUT);
		else
			omap_mux_init_gpio(ccgx_gpio_export[n].gpio,
						OMAP_PIN_OUTPUT);
	}
}

static const char *model = "Color Control GX";
static const char *compat = "technexion,am3517-tam3517ti,am3517ti,omap3";

static ssize_t kobj_read_model(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s", model);
}

static ssize_t kobj_read_compat(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%s", compat);
}

static struct kobj_attribute model_attribute =__ATTR(model, 0660, kobj_read_model, NULL);
static struct kobj_attribute compat_attribute =__ATTR(compatible, 0660, kobj_read_compat, NULL);

static int proc_read_model(char *buf, char **start, off_t offset, int count, int *eof, void *data)
{
	return sprintf(buf, model);
}

static int proc_read_compat(char *buf, char **start, off_t offset, int count, int *eof, void *data)
{
	return sprintf(buf, compat);
}

/* Create fake devicetree entries to compatible with dtb kernels */
static void __init add_device_tree_paths(void)
{
	int error;

	struct kobject *dt, *base;

	dt = kobject_create_and_add("devicetree", firmware_kobj);
	if (!dt) {
		pr_debug("failed to create devicetree\n");
		return;
	}

	base = kobject_create_and_add("base", dt);
	if (!base) {
		pr_debug("failed to create base\n");
		return;
	}

	error = sysfs_create_file(base, &model_attribute.attr);
	if (error)
		pr_debug("failed to create model_kobj %d\n", error);

	error = sysfs_create_file(base, &compat_attribute.attr);
	if (error)
		pr_debug("failed to create compat_kobj %d\n", error);

	create_proc_read_entry("device-tree/model", 0, NULL, proc_read_model, NULL);
	create_proc_read_entry("device-tree/compatible", 0, NULL, proc_read_compat, NULL);
}

static void __init ccgx_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_ZCN);
	set_gpio_key_mux();
	ccgx_i2c_init();

	/*
	 * Keep the left button high. If this is not done the board will
	 * reset, since the F2 button powers the board and loses power
	 * when F1 and F2 are pressed.
	 */
	gpio_request(26, "F1");
	omap_mux_init_gpio(26, OMAP_PIN_INPUT_PULLUP);
	gpio_free(26);

	platform_add_devices(ccgx_devices, ARRAY_SIZE(ccgx_devices));
	ccgx_serial_init();
	omap_sdrc_init(NULL, NULL);

	ccgx_display_init();

	/* CAN */
	am3517_init_hecc();

	/* Configure EHCI ports */
	omap_mux_init_gpio(USB_PHY1_RESET, OMAP_PIN_OUTPUT);
	usbhs_init(&usbhs_bdata);

	/* NAND */
	omap_nand_flash_init(NAND_BUSWIDTH_16, ccgx_nand_partitions,
			     ARRAY_SIZE(ccgx_nand_partitions));

	/* Ethernet */
	am35xx_emac_init(AM35XX_DEFAULT_MDIO_FREQUENCY, 1);

	/* MMC init */
	omap_mux_init_signal("sdmmc1_dat4.gpio_126", OMAP_PIN_INPUT);
	omap_mux_init_signal("cam_strobe.safe_mode", OMAP_PIN_OFF_NONE);
	omap_hsmmc_init(mmc);

	/* Mux init for HW revision input pins */
	omap_mux_init_signal("ccdc_pclk.gpio_94", OMAP_PIN_INPUT);
	omap_mux_init_signal("ccdc_hd.gpio_96", OMAP_PIN_INPUT);
	omap_mux_init_signal("ccdc_wen.gpio_98", OMAP_PIN_INPUT);

	/* USB Peripheral */
	am3517_musb_init();

	ccgx_export_gpio();

	add_device_tree_paths();
}

static void __init init_late(void)
{
	backlight_init();
	am35xx_init_late();
}

MACHINE_START(AM3517_CCGX, "Victron BPP3")
	.atag_offset	= 0x100,
	.reserve	= omap_reserve,
	.map_io		= omap3_map_io,
	.init_early	= am35xx_init_early,
	.init_irq	= omap3_init_irq,
	.handle_irq	= omap3_intc_handle_irq,
	.init_machine	= ccgx_init,
	.init_late	= init_late,
	.timer		= &omap3_timer,
	.restart	= omap_prcm_restart,
MACHINE_END
