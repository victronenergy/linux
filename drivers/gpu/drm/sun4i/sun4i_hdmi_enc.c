// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2016 Maxime Ripard
 *
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 */

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

#include <drm/display/drm_hdmi_helper.h>
#include <drm/display/drm_hdmi_state_helper.h>

#include "sun4i_backend.h"
#include "sun4i_crtc.h"
#include "sun4i_drv.h"
#include "sun4i_hdmi.h"

#define drm_encoder_to_sun4i_hdmi(e)		\
	container_of_const(e, struct sun4i_hdmi, encoder)

#define drm_connector_to_sun4i_hdmi(c)		\
	container_of_const(c, struct sun4i_hdmi, connector)

static int sun4i_hdmi_write_infoframe(struct drm_connector *connector,
				      enum hdmi_infoframe_type type,
				      const u8 *buffer, size_t len)
{
	struct sun4i_hdmi *hdmi = drm_connector_to_sun4i_hdmi(connector);
	int i;

	if (type != HDMI_INFOFRAME_TYPE_AVI) {
		drm_err(connector->dev,
			"Unsupported infoframe type: %u\n", type);
		return 0;
	}

	for (i = 0; i < len; i++)
		writeb(buffer[i], hdmi->base + SUN4I_HDMI_AVI_INFOFRAME_REG(i));

	return 0;

}

static void sun4i_hdmi_disable(struct drm_encoder *encoder,
			       struct drm_atomic_state *state)
{
	struct sun4i_hdmi *hdmi = drm_encoder_to_sun4i_hdmi(encoder);
	u32 val;

	DRM_DEBUG_DRIVER("Disabling the HDMI Output\n");

	val = readl(hdmi->base + SUN4I_HDMI_VID_CTRL_REG);
	val &= ~SUN4I_HDMI_VID_CTRL_ENABLE;
	writel(val, hdmi->base + SUN4I_HDMI_VID_CTRL_REG);

	clk_disable_unprepare(hdmi->tmds_clk);
}

static void sun4i_hdmi_enable(struct drm_encoder *encoder,
			      struct drm_atomic_state *state)
{
	struct drm_display_mode *mode = &encoder->crtc->state->adjusted_mode;
	struct sun4i_hdmi *hdmi = drm_encoder_to_sun4i_hdmi(encoder);
	struct drm_connector *connector = &hdmi->connector;
	struct drm_display_info *display = &connector->display_info;
	struct drm_connector_state *conn_state =
		drm_atomic_get_new_connector_state(state, connector);
	unsigned long long tmds_rate = conn_state->hdmi.tmds_char_rate;
	unsigned int x, y;
	u32 val = 0;

	DRM_DEBUG_DRIVER("Enabling the HDMI Output\n");

	clk_set_rate(hdmi->mod_clk, tmds_rate);
	clk_set_rate(hdmi->tmds_clk, tmds_rate);

	/* Set input sync enable */
	writel(SUN4I_HDMI_UNKNOWN_INPUT_SYNC,
	       hdmi->base + SUN4I_HDMI_UNKNOWN_REG);

	/*
	 * Setup output pad (?) controls
	 *
	 * This is done here instead of at probe/bind time because
	 * the controller seems to toggle some of the bits on its own.
	 *
	 * We can't just initialize the register there, we need to
	 * protect the clock bits that have already been read out and
	 * cached by the clock framework.
	 */
	val = readl(hdmi->base + SUN4I_HDMI_PAD_CTRL1_REG);
	val &= SUN4I_HDMI_PAD_CTRL1_HALVE_CLK;
	val |= hdmi->variant->pad_ctrl1_init_val;
	writel(val, hdmi->base + SUN4I_HDMI_PAD_CTRL1_REG);
	val = readl(hdmi->base + SUN4I_HDMI_PAD_CTRL1_REG);

	/* Setup timing registers */
	writel(SUN4I_HDMI_VID_TIMING_X(mode->hdisplay) |
	       SUN4I_HDMI_VID_TIMING_Y(mode->vdisplay),
	       hdmi->base + SUN4I_HDMI_VID_TIMING_ACT_REG);

	x = mode->htotal - mode->hsync_start;
	y = mode->vtotal - mode->vsync_start;
	writel(SUN4I_HDMI_VID_TIMING_X(x) | SUN4I_HDMI_VID_TIMING_Y(y),
	       hdmi->base + SUN4I_HDMI_VID_TIMING_BP_REG);

	x = mode->hsync_start - mode->hdisplay;
	y = mode->vsync_start - mode->vdisplay;
	writel(SUN4I_HDMI_VID_TIMING_X(x) | SUN4I_HDMI_VID_TIMING_Y(y),
	       hdmi->base + SUN4I_HDMI_VID_TIMING_FP_REG);

	x = mode->hsync_end - mode->hsync_start;
	y = mode->vsync_end - mode->vsync_start;
	writel(SUN4I_HDMI_VID_TIMING_X(x) | SUN4I_HDMI_VID_TIMING_Y(y),
	       hdmi->base + SUN4I_HDMI_VID_TIMING_SPW_REG);

	val = SUN4I_HDMI_VID_TIMING_POL_TX_CLK;
	if (mode->flags & DRM_MODE_FLAG_PHSYNC)
		val |= SUN4I_HDMI_VID_TIMING_POL_HSYNC;

	if (mode->flags & DRM_MODE_FLAG_PVSYNC)
		val |= SUN4I_HDMI_VID_TIMING_POL_VSYNC;

	writel(val, hdmi->base + SUN4I_HDMI_VID_TIMING_POL_REG);

	clk_prepare_enable(hdmi->tmds_clk);

	drm_atomic_helper_connector_hdmi_update_infoframes(connector, state);

	val |= SUN4I_HDMI_PKT_CTRL_TYPE(0, SUN4I_HDMI_PKT_AVI);
	val |= SUN4I_HDMI_PKT_CTRL_TYPE(1, SUN4I_HDMI_PKT_END);
	writel(val, hdmi->base + SUN4I_HDMI_PKT_CTRL_REG(0));

	val = SUN4I_HDMI_VID_CTRL_ENABLE;
	if (display->is_hdmi)
		val |= SUN4I_HDMI_VID_CTRL_HDMI_MODE;

	writel(val, hdmi->base + SUN4I_HDMI_VID_CTRL_REG);
}

static const struct drm_encoder_helper_funcs sun4i_hdmi_helper_funcs = {
	.atomic_disable	= sun4i_hdmi_disable,
	.atomic_enable	= sun4i_hdmi_enable,
};

static enum drm_mode_status
sun4i_hdmi_connector_clock_valid(const struct drm_connector *connector,
				 const struct drm_display_mode *mode,
				 unsigned long long clock)
{
	const struct sun4i_hdmi *hdmi = drm_connector_to_sun4i_hdmi(connector);
	unsigned long diff = div_u64(clock, 200); /* +-0.5% allowed by HDMI spec */
	long rounded_rate;

	if (mode->flags & DRM_MODE_FLAG_DBLCLK)
		return MODE_BAD;

	/* 165 MHz is the typical max pixelclock frequency for HDMI <= 1.2 */
	if (clock > 165000000)
		return MODE_CLOCK_HIGH;

	rounded_rate = clk_round_rate(hdmi->tmds_clk, clock);
	if (rounded_rate > 0 &&
	    max_t(unsigned long, rounded_rate, clock) -
	    min_t(unsigned long, rounded_rate, clock) < diff)
		return MODE_OK;

	return MODE_NOCLOCK;
}

static int sun4i_hdmi_get_modes(struct drm_connector *connector)
{
	struct sun4i_hdmi *hdmi = drm_connector_to_sun4i_hdmi(connector);
	const struct drm_edid *drm_edid;
	int ret;

	drm_edid = drm_edid_read_ddc(connector, hdmi->ddc_i2c ?: hdmi->i2c);

	drm_edid_connector_update(connector, drm_edid);
	cec_s_phys_addr(hdmi->cec_adap,
			connector->display_info.source_physical_address, false);

	if (!drm_edid)
		return 0;

	DRM_DEBUG_DRIVER("Monitor is %s monitor\n",
			 connector->display_info.is_hdmi ? "an HDMI" : "a DVI");


	ret = drm_edid_connector_add_modes(connector);
	drm_edid_free(drm_edid);

	return ret;
}

static struct i2c_adapter *sun4i_hdmi_get_ddc(struct device *dev)
{
	struct device_node *phandle, *remote;
	struct i2c_adapter *ddc;

	remote = of_graph_get_remote_node(dev->of_node, 1, -1);
	if (!remote)
		return ERR_PTR(-EINVAL);

	phandle = of_parse_phandle(remote, "ddc-i2c-bus", 0);
	of_node_put(remote);
	if (!phandle)
		return ERR_PTR(-ENODEV);

	ddc = of_get_i2c_adapter_by_node(phandle);
	of_node_put(phandle);
	if (!ddc)
		return ERR_PTR(-EPROBE_DEFER);

	return ddc;
}

static const struct drm_connector_hdmi_funcs sun4i_hdmi_hdmi_connector_funcs = {
	.tmds_char_rate_valid	= sun4i_hdmi_connector_clock_valid,
	.write_infoframe	= sun4i_hdmi_write_infoframe,
};

static const struct drm_connector_helper_funcs sun4i_hdmi_connector_helper_funcs = {
	.atomic_check	= drm_atomic_helper_connector_hdmi_check,
	.mode_valid	= drm_hdmi_connector_mode_valid,
	.get_modes	= sun4i_hdmi_get_modes,
};

static enum drm_connector_status
sun4i_hdmi_connector_detect(struct drm_connector *connector, bool force)
{
	struct sun4i_hdmi *hdmi = drm_connector_to_sun4i_hdmi(connector);
	unsigned long reg;

	reg = readl(hdmi->base + SUN4I_HDMI_HPD_REG);
	if (!(reg & SUN4I_HDMI_HPD_HIGH)) {
		cec_phys_addr_invalidate(hdmi->cec_adap);
		return connector_status_disconnected;
	}

	return connector_status_connected;
}

static void sun4i_hdmi_connector_reset(struct drm_connector *connector)
{
	drm_atomic_helper_connector_reset(connector);
	__drm_atomic_helper_connector_hdmi_reset(connector, connector->state);
}

static const struct drm_connector_funcs sun4i_hdmi_connector_funcs = {
	.detect			= sun4i_hdmi_connector_detect,
	.fill_modes		= drm_helper_probe_single_connector_modes,
	.reset			= sun4i_hdmi_connector_reset,
	.atomic_duplicate_state	= drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_connector_destroy_state,
};

#ifdef CONFIG_DRM_SUN4I_HDMI_CEC
static int sun4i_hdmi_cec_pin_read(struct cec_adapter *adap)
{
	struct sun4i_hdmi *hdmi = cec_get_drvdata(adap);

	return readl(hdmi->base + SUN4I_HDMI_CEC) & SUN4I_HDMI_CEC_RX;
}

static void sun4i_hdmi_cec_pin_low(struct cec_adapter *adap)
{
	struct sun4i_hdmi *hdmi = cec_get_drvdata(adap);

	/* Start driving the CEC pin low */
	writel(SUN4I_HDMI_CEC_ENABLE, hdmi->base + SUN4I_HDMI_CEC);
}

static void sun4i_hdmi_cec_pin_high(struct cec_adapter *adap)
{
	struct sun4i_hdmi *hdmi = cec_get_drvdata(adap);

	/*
	 * Stop driving the CEC pin, the pull up will take over
	 * unless another CEC device is driving the pin low.
	 */
	writel(0, hdmi->base + SUN4I_HDMI_CEC);
}

static const struct cec_pin_ops sun4i_hdmi_cec_pin_ops = {
	.read = sun4i_hdmi_cec_pin_read,
	.low = sun4i_hdmi_cec_pin_low,
	.high = sun4i_hdmi_cec_pin_high,
};
#endif

#define SUN4I_HDMI_PAD_CTRL1_MASK	(GENMASK(24, 7) | GENMASK(5, 0))
#define SUN4I_HDMI_PLL_CTRL_MASK	(GENMASK(31, 8) | GENMASK(3, 0))

/* Only difference from sun5i is AMP is 4 instead of 6 */
static const struct sun4i_hdmi_variant sun4i_variant = {
	.pad_ctrl0_init_val	= SUN4I_HDMI_PAD_CTRL0_TXEN |
				  SUN4I_HDMI_PAD_CTRL0_CKEN |
				  SUN4I_HDMI_PAD_CTRL0_PWENG |
				  SUN4I_HDMI_PAD_CTRL0_PWEND |
				  SUN4I_HDMI_PAD_CTRL0_PWENC |
				  SUN4I_HDMI_PAD_CTRL0_LDODEN |
				  SUN4I_HDMI_PAD_CTRL0_LDOCEN |
				  SUN4I_HDMI_PAD_CTRL0_BIASEN,
	.pad_ctrl1_init_val	= SUN4I_HDMI_PAD_CTRL1_REG_AMP(4) |
				  SUN4I_HDMI_PAD_CTRL1_REG_EMP(2) |
				  SUN4I_HDMI_PAD_CTRL1_REG_DENCK |
				  SUN4I_HDMI_PAD_CTRL1_REG_DEN |
				  SUN4I_HDMI_PAD_CTRL1_EMPCK_OPT |
				  SUN4I_HDMI_PAD_CTRL1_EMP_OPT |
				  SUN4I_HDMI_PAD_CTRL1_AMPCK_OPT |
				  SUN4I_HDMI_PAD_CTRL1_AMP_OPT,
	.pll_ctrl_init_val	= SUN4I_HDMI_PLL_CTRL_VCO_S(8) |
				  SUN4I_HDMI_PLL_CTRL_CS(7) |
				  SUN4I_HDMI_PLL_CTRL_CP_S(15) |
				  SUN4I_HDMI_PLL_CTRL_S(7) |
				  SUN4I_HDMI_PLL_CTRL_VCO_GAIN(4) |
				  SUN4I_HDMI_PLL_CTRL_SDIV2 |
				  SUN4I_HDMI_PLL_CTRL_LDO2_EN |
				  SUN4I_HDMI_PLL_CTRL_LDO1_EN |
				  SUN4I_HDMI_PLL_CTRL_HV_IS_33 |
				  SUN4I_HDMI_PLL_CTRL_BWS |
				  SUN4I_HDMI_PLL_CTRL_PLL_EN,

	.ddc_clk_reg		= REG_FIELD(SUN4I_HDMI_DDC_CLK_REG, 0, 6),
	.ddc_clk_pre_divider	= 2,
	.ddc_clk_m_offset	= 1,

	.field_ddc_en		= REG_FIELD(SUN4I_HDMI_DDC_CTRL_REG, 31, 31),
	.field_ddc_start	= REG_FIELD(SUN4I_HDMI_DDC_CTRL_REG, 30, 30),
	.field_ddc_reset	= REG_FIELD(SUN4I_HDMI_DDC_CTRL_REG, 0, 0),
	.field_ddc_addr_reg	= REG_FIELD(SUN4I_HDMI_DDC_ADDR_REG, 0, 31),
	.field_ddc_slave_addr	= REG_FIELD(SUN4I_HDMI_DDC_ADDR_REG, 0, 6),
	.field_ddc_int_status	= REG_FIELD(SUN4I_HDMI_DDC_INT_STATUS_REG, 0, 8),
	.field_ddc_fifo_clear	= REG_FIELD(SUN4I_HDMI_DDC_FIFO_CTRL_REG, 31, 31),
	.field_ddc_fifo_rx_thres = REG_FIELD(SUN4I_HDMI_DDC_FIFO_CTRL_REG, 4, 7),
	.field_ddc_fifo_tx_thres = REG_FIELD(SUN4I_HDMI_DDC_FIFO_CTRL_REG, 0, 3),
	.field_ddc_byte_count	= REG_FIELD(SUN4I_HDMI_DDC_BYTE_COUNT_REG, 0, 9),
	.field_ddc_cmd		= REG_FIELD(SUN4I_HDMI_DDC_CMD_REG, 0, 2),
	.field_ddc_sda_en	= REG_FIELD(SUN4I_HDMI_DDC_LINE_CTRL_REG, 9, 9),
	.field_ddc_sck_en	= REG_FIELD(SUN4I_HDMI_DDC_LINE_CTRL_REG, 8, 8),

	.ddc_fifo_reg		= SUN4I_HDMI_DDC_FIFO_DATA_REG,
	.ddc_fifo_has_dir	= true,
};

static const struct sun4i_hdmi_variant sun5i_variant = {
	.pad_ctrl0_init_val	= SUN4I_HDMI_PAD_CTRL0_TXEN |
				  SUN4I_HDMI_PAD_CTRL0_CKEN |
				  SUN4I_HDMI_PAD_CTRL0_PWENG |
				  SUN4I_HDMI_PAD_CTRL0_PWEND |
				  SUN4I_HDMI_PAD_CTRL0_PWENC |
				  SUN4I_HDMI_PAD_CTRL0_LDODEN |
				  SUN4I_HDMI_PAD_CTRL0_LDOCEN |
				  SUN4I_HDMI_PAD_CTRL0_BIASEN,
	.pad_ctrl1_init_val	= SUN4I_HDMI_PAD_CTRL1_REG_AMP(6) |
				  SUN4I_HDMI_PAD_CTRL1_REG_EMP(2) |
				  SUN4I_HDMI_PAD_CTRL1_REG_DENCK |
				  SUN4I_HDMI_PAD_CTRL1_REG_DEN |
				  SUN4I_HDMI_PAD_CTRL1_EMPCK_OPT |
				  SUN4I_HDMI_PAD_CTRL1_EMP_OPT |
				  SUN4I_HDMI_PAD_CTRL1_AMPCK_OPT |
				  SUN4I_HDMI_PAD_CTRL1_AMP_OPT,
	.pll_ctrl_init_val	= SUN4I_HDMI_PLL_CTRL_VCO_S(8) |
				  SUN4I_HDMI_PLL_CTRL_CS(7) |
				  SUN4I_HDMI_PLL_CTRL_CP_S(15) |
				  SUN4I_HDMI_PLL_CTRL_S(7) |
				  SUN4I_HDMI_PLL_CTRL_VCO_GAIN(4) |
				  SUN4I_HDMI_PLL_CTRL_SDIV2 |
				  SUN4I_HDMI_PLL_CTRL_LDO2_EN |
				  SUN4I_HDMI_PLL_CTRL_LDO1_EN |
				  SUN4I_HDMI_PLL_CTRL_HV_IS_33 |
				  SUN4I_HDMI_PLL_CTRL_BWS |
				  SUN4I_HDMI_PLL_CTRL_PLL_EN,

	.ddc_clk_reg		= REG_FIELD(SUN4I_HDMI_DDC_CLK_REG, 0, 6),
	.ddc_clk_pre_divider	= 2,
	.ddc_clk_m_offset	= 1,

	.field_ddc_en		= REG_FIELD(SUN4I_HDMI_DDC_CTRL_REG, 31, 31),
	.field_ddc_start	= REG_FIELD(SUN4I_HDMI_DDC_CTRL_REG, 30, 30),
	.field_ddc_reset	= REG_FIELD(SUN4I_HDMI_DDC_CTRL_REG, 0, 0),
	.field_ddc_addr_reg	= REG_FIELD(SUN4I_HDMI_DDC_ADDR_REG, 0, 31),
	.field_ddc_slave_addr	= REG_FIELD(SUN4I_HDMI_DDC_ADDR_REG, 0, 6),
	.field_ddc_int_status	= REG_FIELD(SUN4I_HDMI_DDC_INT_STATUS_REG, 0, 8),
	.field_ddc_fifo_clear	= REG_FIELD(SUN4I_HDMI_DDC_FIFO_CTRL_REG, 31, 31),
	.field_ddc_fifo_rx_thres = REG_FIELD(SUN4I_HDMI_DDC_FIFO_CTRL_REG, 4, 7),
	.field_ddc_fifo_tx_thres = REG_FIELD(SUN4I_HDMI_DDC_FIFO_CTRL_REG, 0, 3),
	.field_ddc_byte_count	= REG_FIELD(SUN4I_HDMI_DDC_BYTE_COUNT_REG, 0, 9),
	.field_ddc_cmd		= REG_FIELD(SUN4I_HDMI_DDC_CMD_REG, 0, 2),
	.field_ddc_sda_en	= REG_FIELD(SUN4I_HDMI_DDC_LINE_CTRL_REG, 9, 9),
	.field_ddc_sck_en	= REG_FIELD(SUN4I_HDMI_DDC_LINE_CTRL_REG, 8, 8),

	.ddc_fifo_reg		= SUN4I_HDMI_DDC_FIFO_DATA_REG,
	.ddc_fifo_has_dir	= true,
};

static const struct sun4i_hdmi_variant sun6i_variant = {
	.has_ddc_parent_clk	= true,
	.has_reset_control	= true,
	.pad_ctrl0_init_val	= 0xff |
				  SUN4I_HDMI_PAD_CTRL0_TXEN |
				  SUN4I_HDMI_PAD_CTRL0_CKEN |
				  SUN4I_HDMI_PAD_CTRL0_PWENG |
				  SUN4I_HDMI_PAD_CTRL0_PWEND |
				  SUN4I_HDMI_PAD_CTRL0_PWENC |
				  SUN4I_HDMI_PAD_CTRL0_LDODEN |
				  SUN4I_HDMI_PAD_CTRL0_LDOCEN,
	.pad_ctrl1_init_val	= SUN4I_HDMI_PAD_CTRL1_REG_AMP(6) |
				  SUN4I_HDMI_PAD_CTRL1_REG_EMP(4) |
				  SUN4I_HDMI_PAD_CTRL1_REG_DENCK |
				  SUN4I_HDMI_PAD_CTRL1_REG_DEN |
				  SUN4I_HDMI_PAD_CTRL1_EMPCK_OPT |
				  SUN4I_HDMI_PAD_CTRL1_EMP_OPT |
				  SUN4I_HDMI_PAD_CTRL1_PWSDT |
				  SUN4I_HDMI_PAD_CTRL1_PWSCK |
				  SUN4I_HDMI_PAD_CTRL1_AMPCK_OPT |
				  SUN4I_HDMI_PAD_CTRL1_AMP_OPT |
				  SUN4I_HDMI_PAD_CTRL1_UNKNOWN,
	.pll_ctrl_init_val	= SUN4I_HDMI_PLL_CTRL_VCO_S(8) |
				  SUN4I_HDMI_PLL_CTRL_CS(3) |
				  SUN4I_HDMI_PLL_CTRL_CP_S(10) |
				  SUN4I_HDMI_PLL_CTRL_S(4) |
				  SUN4I_HDMI_PLL_CTRL_VCO_GAIN(4) |
				  SUN4I_HDMI_PLL_CTRL_SDIV2 |
				  SUN4I_HDMI_PLL_CTRL_LDO2_EN |
				  SUN4I_HDMI_PLL_CTRL_LDO1_EN |
				  SUN4I_HDMI_PLL_CTRL_HV_IS_33 |
				  SUN4I_HDMI_PLL_CTRL_PLL_EN,

	.ddc_clk_reg		= REG_FIELD(SUN6I_HDMI_DDC_CLK_REG, 0, 6),
	.ddc_clk_pre_divider	= 1,
	.ddc_clk_m_offset	= 2,

	.tmds_clk_div_offset	= 1,

	.field_ddc_en		= REG_FIELD(SUN6I_HDMI_DDC_CTRL_REG, 0, 0),
	.field_ddc_start	= REG_FIELD(SUN6I_HDMI_DDC_CTRL_REG, 27, 27),
	.field_ddc_reset	= REG_FIELD(SUN6I_HDMI_DDC_CTRL_REG, 31, 31),
	.field_ddc_addr_reg	= REG_FIELD(SUN6I_HDMI_DDC_ADDR_REG, 1, 31),
	.field_ddc_slave_addr	= REG_FIELD(SUN6I_HDMI_DDC_ADDR_REG, 1, 7),
	.field_ddc_int_status	= REG_FIELD(SUN6I_HDMI_DDC_INT_STATUS_REG, 0, 8),
	.field_ddc_fifo_clear	= REG_FIELD(SUN6I_HDMI_DDC_FIFO_CTRL_REG, 18, 18),
	.field_ddc_fifo_rx_thres = REG_FIELD(SUN6I_HDMI_DDC_FIFO_CTRL_REG, 4, 7),
	.field_ddc_fifo_tx_thres = REG_FIELD(SUN6I_HDMI_DDC_FIFO_CTRL_REG, 0, 3),
	.field_ddc_byte_count	= REG_FIELD(SUN6I_HDMI_DDC_CMD_REG, 16, 25),
	.field_ddc_cmd		= REG_FIELD(SUN6I_HDMI_DDC_CMD_REG, 0, 2),
	.field_ddc_sda_en	= REG_FIELD(SUN6I_HDMI_DDC_CTRL_REG, 6, 6),
	.field_ddc_sck_en	= REG_FIELD(SUN6I_HDMI_DDC_CTRL_REG, 4, 4),

	.ddc_fifo_reg		= SUN6I_HDMI_DDC_FIFO_DATA_REG,
	.ddc_fifo_thres_incl	= true,
};

static const struct regmap_config sun4i_hdmi_regmap_config = {
	.reg_bits	= 32,
	.val_bits	= 32,
	.reg_stride	= 4,
	.max_register	= 0x580,
};

static int sun4i_hdmi_bind(struct device *dev, struct device *master,
			   void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct drm_device *drm = data;
	struct cec_connector_info conn_info;
	struct sun4i_drv *drv = drm->dev_private;
	struct sun4i_hdmi *hdmi;
	u32 reg;
	int ret;

	hdmi = devm_kzalloc(dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;
	dev_set_drvdata(dev, hdmi);
	hdmi->dev = dev;
	hdmi->drv = drv;

	hdmi->variant = of_device_get_match_data(dev);
	if (!hdmi->variant)
		return -EINVAL;

	hdmi->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(hdmi->base)) {
		dev_err(dev, "Couldn't map the HDMI encoder registers\n");
		return PTR_ERR(hdmi->base);
	}

	if (hdmi->variant->has_reset_control) {
		hdmi->reset = devm_reset_control_get(dev, NULL);
		if (IS_ERR(hdmi->reset)) {
			dev_err(dev, "Couldn't get the HDMI reset control\n");
			return PTR_ERR(hdmi->reset);
		}

		ret = reset_control_deassert(hdmi->reset);
		if (ret) {
			dev_err(dev, "Couldn't deassert HDMI reset\n");
			return ret;
		}
	}

	hdmi->bus_clk = devm_clk_get(dev, "ahb");
	if (IS_ERR(hdmi->bus_clk)) {
		dev_err(dev, "Couldn't get the HDMI bus clock\n");
		ret = PTR_ERR(hdmi->bus_clk);
		goto err_assert_reset;
	}
	clk_prepare_enable(hdmi->bus_clk);

	hdmi->mod_clk = devm_clk_get(dev, "mod");
	if (IS_ERR(hdmi->mod_clk)) {
		dev_err(dev, "Couldn't get the HDMI mod clock\n");
		ret = PTR_ERR(hdmi->mod_clk);
		goto err_disable_bus_clk;
	}
	clk_prepare_enable(hdmi->mod_clk);

	hdmi->pll0_clk = devm_clk_get(dev, "pll-0");
	if (IS_ERR(hdmi->pll0_clk)) {
		dev_err(dev, "Couldn't get the HDMI PLL 0 clock\n");
		ret = PTR_ERR(hdmi->pll0_clk);
		goto err_disable_mod_clk;
	}

	hdmi->pll1_clk = devm_clk_get(dev, "pll-1");
	if (IS_ERR(hdmi->pll1_clk)) {
		dev_err(dev, "Couldn't get the HDMI PLL 1 clock\n");
		ret = PTR_ERR(hdmi->pll1_clk);
		goto err_disable_mod_clk;
	}

	hdmi->regmap = devm_regmap_init_mmio(dev, hdmi->base,
					     &sun4i_hdmi_regmap_config);
	if (IS_ERR(hdmi->regmap)) {
		dev_err(dev, "Couldn't create HDMI encoder regmap\n");
		ret = PTR_ERR(hdmi->regmap);
		goto err_disable_mod_clk;
	}

	ret = sun4i_tmds_create(hdmi);
	if (ret) {
		dev_err(dev, "Couldn't create the TMDS clock\n");
		goto err_disable_mod_clk;
	}

	if (hdmi->variant->has_ddc_parent_clk) {
		hdmi->ddc_parent_clk = devm_clk_get(dev, "ddc");
		if (IS_ERR(hdmi->ddc_parent_clk)) {
			dev_err(dev, "Couldn't get the HDMI DDC clock\n");
			ret = PTR_ERR(hdmi->ddc_parent_clk);
			goto err_disable_mod_clk;
		}
	} else {
		hdmi->ddc_parent_clk = hdmi->tmds_clk;
	}

	writel(SUN4I_HDMI_CTRL_ENABLE, hdmi->base + SUN4I_HDMI_CTRL_REG);

	writel(hdmi->variant->pad_ctrl0_init_val,
	       hdmi->base + SUN4I_HDMI_PAD_CTRL0_REG);

	reg = readl(hdmi->base + SUN4I_HDMI_PLL_CTRL_REG);
	reg &= SUN4I_HDMI_PLL_CTRL_DIV_MASK;
	reg |= hdmi->variant->pll_ctrl_init_val;
	writel(reg, hdmi->base + SUN4I_HDMI_PLL_CTRL_REG);

	ret = sun4i_hdmi_i2c_create(dev, hdmi);
	if (ret) {
		dev_err(dev, "Couldn't create the HDMI I2C adapter\n");
		goto err_disable_mod_clk;
	}

	hdmi->ddc_i2c = sun4i_hdmi_get_ddc(dev);
	if (IS_ERR(hdmi->ddc_i2c)) {
		ret = PTR_ERR(hdmi->ddc_i2c);
		if (ret == -ENODEV)
			hdmi->ddc_i2c = NULL;
		else
			goto err_del_i2c_adapter;
	}

	drm_encoder_helper_add(&hdmi->encoder,
			       &sun4i_hdmi_helper_funcs);
	ret = drm_simple_encoder_init(drm, &hdmi->encoder,
				      DRM_MODE_ENCODER_TMDS);
	if (ret) {
		dev_err(dev, "Couldn't initialise the HDMI encoder\n");
		goto err_put_ddc_i2c;
	}

	hdmi->encoder.possible_crtcs = drm_of_find_possible_crtcs(drm,
								  dev->of_node);
	if (!hdmi->encoder.possible_crtcs) {
		ret = -EPROBE_DEFER;
		goto err_put_ddc_i2c;
	}

#ifdef CONFIG_DRM_SUN4I_HDMI_CEC
	hdmi->cec_adap = cec_pin_allocate_adapter(&sun4i_hdmi_cec_pin_ops,
		hdmi, "sun4i", CEC_CAP_DEFAULTS | CEC_CAP_CONNECTOR_INFO);
	ret = PTR_ERR_OR_ZERO(hdmi->cec_adap);
	if (ret < 0)
		goto err_cleanup_connector;
	writel(readl(hdmi->base + SUN4I_HDMI_CEC) & ~SUN4I_HDMI_CEC_TX,
	       hdmi->base + SUN4I_HDMI_CEC);
#endif

	drm_connector_helper_add(&hdmi->connector,
				 &sun4i_hdmi_connector_helper_funcs);
	ret = drmm_connector_hdmi_init(drm, &hdmi->connector,
				       /*
					* NOTE: Those are likely to be
					* wrong, but I couldn't find the
					* actual ones in the BSP.
					*/
				       "AW", "HDMI",
				       &sun4i_hdmi_connector_funcs,
				       &sun4i_hdmi_hdmi_connector_funcs,
				       DRM_MODE_CONNECTOR_HDMIA,
				       hdmi->ddc_i2c,
				       BIT(HDMI_COLORSPACE_RGB),
				       8);
	if (ret) {
		dev_err(dev,
			"Couldn't initialise the HDMI connector\n");
		goto err_cleanup_connector;
	}
	cec_fill_conn_info_from_drm(&conn_info, &hdmi->connector);
	cec_s_conn_info(hdmi->cec_adap, &conn_info);

	/* There is no HPD interrupt, so we need to poll the controller */
	hdmi->connector.polled = DRM_CONNECTOR_POLL_CONNECT |
		DRM_CONNECTOR_POLL_DISCONNECT;

	ret = cec_register_adapter(hdmi->cec_adap, dev);
	if (ret < 0)
		goto err_cleanup_connector;
	drm_connector_attach_encoder(&hdmi->connector, &hdmi->encoder);

	return 0;

err_cleanup_connector:
	cec_delete_adapter(hdmi->cec_adap);
	drm_encoder_cleanup(&hdmi->encoder);
err_put_ddc_i2c:
	i2c_put_adapter(hdmi->ddc_i2c);
err_del_i2c_adapter:
	i2c_del_adapter(hdmi->i2c);
err_disable_mod_clk:
	clk_disable_unprepare(hdmi->mod_clk);
err_disable_bus_clk:
	clk_disable_unprepare(hdmi->bus_clk);
err_assert_reset:
	reset_control_assert(hdmi->reset);
	return ret;
}

static void sun4i_hdmi_unbind(struct device *dev, struct device *master,
			    void *data)
{
	struct sun4i_hdmi *hdmi = dev_get_drvdata(dev);

	cec_unregister_adapter(hdmi->cec_adap);
	i2c_del_adapter(hdmi->i2c);
	i2c_put_adapter(hdmi->ddc_i2c);
	clk_disable_unprepare(hdmi->mod_clk);
	clk_disable_unprepare(hdmi->bus_clk);
}

static const struct component_ops sun4i_hdmi_ops = {
	.bind	= sun4i_hdmi_bind,
	.unbind	= sun4i_hdmi_unbind,
};

static int sun4i_hdmi_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &sun4i_hdmi_ops);
}

static void sun4i_hdmi_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &sun4i_hdmi_ops);
}

static const struct of_device_id sun4i_hdmi_of_table[] = {
	{ .compatible = "allwinner,sun4i-a10-hdmi", .data = &sun4i_variant, },
	{ .compatible = "allwinner,sun5i-a10s-hdmi", .data = &sun5i_variant, },
	{ .compatible = "allwinner,sun6i-a31-hdmi", .data = &sun6i_variant, },
	{ }
};
MODULE_DEVICE_TABLE(of, sun4i_hdmi_of_table);

static struct platform_driver sun4i_hdmi_driver = {
	.probe		= sun4i_hdmi_probe,
	.remove_new	= sun4i_hdmi_remove,
	.driver		= {
		.name		= "sun4i-hdmi",
		.of_match_table	= sun4i_hdmi_of_table,
	},
};
module_platform_driver(sun4i_hdmi_driver);

MODULE_AUTHOR("Maxime Ripard <maxime.ripard@free-electrons.com>");
MODULE_DESCRIPTION("Allwinner A10 HDMI Driver");
MODULE_LICENSE("GPL");
