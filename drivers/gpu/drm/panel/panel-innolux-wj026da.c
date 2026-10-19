// SPDX-License-Identifier: GPL-2.0-only
/*
 * Innolux WJ026DA-03A (NT57900 DDIC) panel driver.
 * Timing/commands: the device's stock runtime DTB. Sequence ordering:
 * bytedance/phoenix-kernel ceafd3afc020, dsi_panel.c and dsi_drm.c.
 *
 * Diagnostic handoff mode preserves the bootloader's already-enabled bias
 * and backlight. It initializes pixel output without claiming cold-start
 * power management or suspend/resume support.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/regulator/consumer.h>

#include <drm/display/drm_dsc.h>
#include <drm/drm_connector.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <video/mipi_display.h>

/*
 * A8110 bring-up: mainline MSM DSI internal helper. The panel and MSM drivers
 * are built into the same image, so use a local prototype instead of pulling
 * in the MSM-private dsi.h (it needs the MSM include path).
 */
int msm_dsi_host_enable(struct mipi_dsi_host *host);

/* A8110 continuous-splash handoff, defined in msm_drv.c. */
extern bool msm_cont_splash;

#include "panel-innolux-wj026da-cmds.h"

/* Stock panel-supply-entries, followed by the external AW3750x bias rails. */
static const char * const wj026da_supply_names[] = {
	"vddio", "vdd", "lab", "ibb", "vsp", "vsn",
};
static const unsigned int wj026da_on_delay[] = { 20, 0, 6, 20, 8, 8 };
static const unsigned int wj026da_off_delay[] = { 0, 0, 6, 12, 8, 2 };

struct wj026da {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi[2];
	struct drm_dsc_config dsc[2];
	struct regulator *supplies[ARRAY_SIZE(wj026da_supply_names)];
	struct gpio_desc *reset;
	struct gpio_desc *vio_enable;
	struct gpio_desc *enp;
	struct gpio_desc *enn;
	unsigned int supplies_on;
	bool p1;
	bool preserve_power;
};

static inline struct wj026da *to_wj026da(struct drm_panel *panel)
{
	return container_of(panel, struct wj026da, panel);
}

/*
 * Send each command to both links, then wait once. On MSM sync-dual-dsi,
 * the manager broadcasts the triggering link's write to both hosts.
 * Keep 0x29 generic LONG writes: mipi_dsi_generic_write() would convert
 * two-byte payloads to short writes, unlike the stock command stream.
 *
 * The first init command occasionally times out on warm boots while the DSI
 * link is settling. Retry the individual command before failing the whole
 * panel init; this is safe in command mode because the video engine is off.
 */
#define WJ026DA_CMD_RETRIES	3

static int wj026da_send(struct wj026da *ctx, const struct wj026da_cmd *cmds,
			unsigned int count, bool lp)
{
	unsigned int i, link;

	for (i = 0; i < count; i++) {
		for (link = 0; link < ARRAY_SIZE(ctx->dsi); link++) {
			struct mipi_dsi_device *dsi = ctx->dsi[link];
			struct mipi_dsi_msg msg = {
				.channel = dsi->channel,
				.type = cmds[i].type,
				.flags = lp ? MIPI_DSI_MSG_USE_LPM : 0,
				.tx_len = cmds[i].len,
				.tx_buf = cmds[i].data,
			};
			ssize_t ret = -EIO;
			unsigned int attempt;

			/* Downstream pads 0x05 off commands with a dummy byte. */
			if (msg.type == MIPI_DSI_DCS_SHORT_WRITE)
				msg.tx_len = 1;

			/* Raw host transfer retains the downstream packet type. */
			if (!dsi->host->ops->transfer)
				return -EOPNOTSUPP;

			for (attempt = 0; attempt < WJ026DA_CMD_RETRIES; attempt++) {
				ret = dsi->host->ops->transfer(dsi->host, &msg);
				if (ret >= 0 && ret >= msg.tx_len)
					break;
				if (attempt + 1 < WJ026DA_CMD_RETRIES) {
					dev_warn(ctx->panel.dev,
						 "DSI%u command %u retry %u (ret=%zd)\n",
						 link, i, attempt + 1, ret);
					usleep_range(5000, 10000);
				}
			}
			if (ret < 0)
				return dev_err_probe(ctx->panel.dev, ret,
						     "DSI%u command %u failed\n", link, i);
			/* MSM reports the padded DMA packet length on success. */
			if (ret < msg.tx_len)
				return dev_err_probe(ctx->panel.dev, -EIO,
						     "DSI%u command %u short transfer: %zd/%zu\n",
						     link, i, ret, msg.tx_len);
		}
		if (cmds[i].delay_ms)
			msleep(cmds[i].delay_ms);
	}
	return 0;
}

static int wj026da_power_off(struct wj026da *ctx)
{
	int ret = 0, err;

	/*
	 * Keep the bootloader's rails/AW3750x/backlight state across
	 * unprepare. Toggling AW3750x off here made the next warm boot come
	 * up with the backlight already off if panel init then failed.
	 */
	if (ctx->preserve_power || msm_cont_splash)
		return 0;

	gpiod_set_value_cansleep(ctx->reset, 1);
	usleep_range(10000, 11000);

	/* AW3750x: ENN off, 2 ms, ENP off, 8 ms (vendor order). */
	gpiod_set_value_cansleep(ctx->enn, 0);
	usleep_range(2000, 2100);
	gpiod_set_value_cansleep(ctx->enp, 0);
	usleep_range(8000, 8100);

	while (ctx->supplies_on) {
		unsigned int i = --ctx->supplies_on;

		err = regulator_disable(ctx->supplies[i]);
		if (err && !ret)
			ret = err;
		if (wj026da_off_delay[i])
			fsleep(wj026da_off_delay[i] * 1000);
	}
	gpiod_set_value_cansleep(ctx->vio_enable, 0);
	return ret;
}

static int wj026da_prepare(struct drm_panel *panel)
{
	struct wj026da *ctx = to_wj026da(panel);
	unsigned int i;
	int ret;

	/*
	 * A8110 continuous-splash handoff: the bootloader already initialized
	 * the panel/DSI/DPU. Do not reset or re-send init/PPS here; the DSI
	 * and DPU paths skip their resets in the same mode.
	 */
	if (msm_cont_splash) {
		dev_info(panel->dev, "adopting bootloader continuous splash\n");
		return 0;
	}

	gpiod_set_value_cansleep(ctx->vio_enable, 1);

	if (!ctx->preserve_power) {
		for (i = 0; i < ARRAY_SIZE(ctx->supplies); i++) {
			ret = regulator_enable(ctx->supplies[i]);
			if (ret)
				goto power_off;
			ctx->supplies_on++;
			if (wj026da_on_delay[i])
				fsleep(wj026da_on_delay[i] * 1000);
		}
	}

	/* AW3750x: 8 ms, ENP on, 8 ms, ENN on, 8 ms (vendor order). */
	msleep(8);
	gpiod_set_value_cansleep(ctx->enp, 1);
	msleep(8);
	gpiod_set_value_cansleep(ctx->enn, 1);
	msleep(8);

	/* Active-low reset; stock physical levels: high20 low20 high120. */
	gpiod_set_value_cansleep(ctx->reset, 0);
	msleep(20);
	gpiod_set_value_cansleep(ctx->reset, 1);
	msleep(20);
	gpiod_set_value_cansleep(ctx->reset, 0);
	msleep(120);

initialize:
	if (ctx->p1)
		ret = wj026da_send(ctx, wj026da_on_p1, ARRAY_SIZE(wj026da_on_p1), true);
	else
		ret = wj026da_send(ctx, wj026da_on, ARRAY_SIZE(wj026da_on), true);
	if (ret)
		goto power_off;

	/*
	 * Do not add a separate PPS/compression write here. The captured ON
	 * sequence already configures the receiver; on console31 the extra
	 * generator-style PPS produced a garbled picture even though init and
	 * the DPU pipeline completed.
	 */

	/*
	 * Send post-90 here, while the DSI link is still in command mode.
	 * The mainline panel enable runs before the DPU encoder enable, so
	 * sending it from ->enable() makes these commands wait for a
	 * video-done interrupt that cannot arrive yet. That is the
	 * probabilistic backlight-only / one-garbled-eye failure.
	 */
	if (!ctx->p1) {
		ret = wj026da_send(ctx, wj026da_post_90,
				   ARRAY_SIZE(wj026da_post_90), true);
		if (ret)
			goto power_off;
	}

	/*
	 * Match the vendor order: panel ON -> DSI video engine -> DPU. The
	 * mainline DRM helper enables the DPU encoder before the DSI bridge
	 * .enable, so without this the DPU CTL reset waits for a DSI video
	 * engine that is not running yet.
	 */
	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
		ret = msm_dsi_host_enable(ctx->dsi[i]->host);
		if (ret) {
			dev_err(&ctx->dsi[i]->dev,
				"failed to enable DSI video: %d\n", ret);
			goto power_off;
		}
	}

	dev_info(panel->dev, "%s init completed on both DSI links\n",
		 ctx->p1 ? "P1" : "production");

	return 0;

power_off:
	wj026da_power_off(ctx);
	return ret;
}

static int wj026da_enable(struct drm_panel *panel)
{
	/*
	 * post-90 is sent from wj026da_prepare(), before DSI video mode is
	 * enabled. The mainline enable hook runs before the DPU encoder
	 * enable, so it cannot send commands that wait for video-done.
	 */
	(void)panel;
	return 0;
}

static int wj026da_disable(struct drm_panel *panel)
{
	return wj026da_send(to_wj026da(panel), wj026da_off, ARRAY_SIZE(wj026da_off), false);
}

static int wj026da_unprepare(struct drm_panel *panel)
{
	return wj026da_power_off(to_wj026da(panel));
}

/* The downstream display code doubles ALL horizontal timings for two links. */
static const struct drm_display_mode wj026da_mode = {
	.clock = (2160 + 161 + 4 + 161) * 2 * (2160 + 48 + 2 + 14) * 90 / 1000,
	.hdisplay = 2160 * 2,
	.hsync_start = (2160 + 161) * 2,
	.hsync_end = (2160 + 161 + 4) * 2,
	.htotal = (2160 + 161 + 4 + 161) * 2,
	.vdisplay = 2160,
	.vsync_start = 2160 + 48,
	.vsync_end = 2160 + 48 + 2,
	.vtotal = 2160 + 48 + 2 + 14,
	.width_mm = 92,
	.height_mm = 46,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static int wj026da_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &wj026da_mode);
	if (!mode)
		return -ENOMEM;
	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);
	connector->display_info.width_mm = wj026da_mode.width_mm;
	connector->display_info.height_mm = wj026da_mode.height_mm;
	connector->display_info.bpc = 8;
	return 1;
}

static const struct drm_panel_funcs wj026da_panel_funcs = {
	.prepare = wj026da_prepare,
	.enable = wj026da_enable,
	.disable = wj026da_disable,
	.unprepare = wj026da_unprepare,
	.get_modes = wj026da_get_modes,
};

static int wj026da_probe(struct mipi_dsi_device *dsi)
{
	const struct mipi_dsi_device_info secondary = { .type = "wj026da-secondary" };
	struct device *dev = &dsi->dev;
	struct device_node *node;
	struct mipi_dsi_host *host;
	struct wj026da *ctx;
	const char *sequence;
	unsigned int i;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct wj026da, panel, &wj026da_panel_funcs,
				 DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ret = of_property_read_string(dev->of_node, "pico,init-sequence", &sequence);
	if (ret)
		return dev_err_probe(dev, ret, "specify verified p1 or production init sequence\n");
	if (!strcmp(sequence, "p1"))
		ctx->p1 = true;
	else if (strcmp(sequence, "production"))
		return -EINVAL;

	ctx->preserve_power = of_property_read_bool(dev->of_node,
						   "pico,preserve-boot-power");
	if (ctx->preserve_power)
		goto secondary_host;

	/* Never let dummy regulators conceal an incomplete cold-start power tree. */
	for (i = 0; i < ARRAY_SIZE(ctx->supplies); i++) {
		char property[24];

		snprintf(property, sizeof(property), "%s-supply", wj026da_supply_names[i]);
		if (!of_find_property(dev->of_node, property, NULL))
			return dev_err_probe(dev, -EINVAL, "missing required %s\n", property);
		ctx->supplies[i] = devm_regulator_get(dev, wj026da_supply_names[i]);
		if (IS_ERR(ctx->supplies[i]))
			return dev_err_probe(dev, PTR_ERR(ctx->supplies[i]),
					     "failed to get %s\n", wj026da_supply_names[i]);
	}
	if (!of_find_property(dev->of_node, "backlight", NULL))
		return dev_err_probe(dev, -EINVAL, "external backlight provider is required\n");
	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return ret;

secondary_host:
	node = of_graph_get_remote_node(dev->of_node, 1, -1);
	if (!node)
		return dev_err_probe(dev, -ENODEV, "missing secondary DSI endpoint\n");
	host = of_find_mipi_dsi_host_by_node(node);
	of_node_put(node);
	if (!host)
		return -EPROBE_DEFER;
	ctx->dsi[1] = devm_mipi_dsi_device_register_full(dev, host, &secondary);
	if (IS_ERR(ctx->dsi[1]))
		return PTR_ERR(ctx->dsi[1]);
	ctx->dsi[0] = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset))
		return dev_err_probe(dev, PTR_ERR(ctx->reset), "failed to get reset GPIO\n");
	ctx->vio_enable = devm_gpiod_get(dev, "vio-enable", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->vio_enable))
		return dev_err_probe(dev, PTR_ERR(ctx->vio_enable), "failed to get VIO GPIO\n");
	ctx->enp = devm_gpiod_get(dev, "enp", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->enp))
		return dev_err_probe(dev, PTR_ERR(ctx->enp), "failed to get ENP GPIO\n");
	ctx->enn = devm_gpiod_get(dev, "enn", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->enn))
		return dev_err_probe(dev, PTR_ERR(ctx->enn), "failed to get ENN GPIO\n");

configure_links:
	ctx->panel.prepare_prev_first = true;
	for (i = 0; i < ARRAY_SIZE(ctx->dsi); i++) {
		ctx->dsc[i].dsc_version_major = 1;
		ctx->dsc[i].dsc_version_minor = 1;
		ctx->dsc[i].slice_height = 16;
		ctx->dsc[i].slice_width = 1080;
		/* Two slices per link; four across the complete stereo frame. */
		ctx->dsc[i].slice_count = 2;
		ctx->dsc[i].dsc_slice_per_pkt = 2;
		ctx->dsc[i].bits_per_component = 8;
		ctx->dsc[i].bits_per_pixel = 8 << 4;
		ctx->dsc[i].block_pred_enable = true;
		ctx->dsi[i]->dsc = &ctx->dsc[i];
		ctx->dsi[i]->dual_panel = true;
		ctx->dsi[i]->lanes = 4;
		ctx->dsi[i]->format = MIPI_DSI_FMT_RGB888;
		/* Set LP/HS per message rather than forcing all messages into LP. */
		ctx->dsi[i]->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
					  MIPI_DSI_CLOCK_NON_CONTINUOUS;
	}

	drm_panel_add(&ctx->panel);
	/*
	 * Attach the secondary link first. The primary attach synchronously
	 * triggers the MSM component bind, which can prepare the panel and
	 * send commands to both links before the primary attach returns.
	 * Both hosts must have their lane/format/DSC configuration by then.
	 */
	for (i = ARRAY_SIZE(ctx->dsi); i-- > 0; ) {
		ret = devm_mipi_dsi_attach(dev, ctx->dsi[i]);
		if (ret) {
			drm_panel_remove(&ctx->panel);
			return dev_err_probe(dev, ret, "failed to attach DSI%u\n", i);
		}
	}
	dev_info(dev, "4320x2160@90 dual panel: 4 DSC, 2 slices/packet, %s power\n",
		 ctx->preserve_power ? "bootloader-preserved" : "managed");
	return 0;
}

static void wj026da_shutdown(struct mipi_dsi_device *dsi)
{
	struct wj026da *ctx = mipi_dsi_get_drvdata(dsi);

	drm_panel_disable(&ctx->panel);
	drm_panel_unprepare(&ctx->panel);
}

static void wj026da_remove(struct mipi_dsi_device *dsi)
{
	struct wj026da *ctx = mipi_dsi_get_drvdata(dsi);

	wj026da_shutdown(dsi);
	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id wj026da_of_match[] = {
	{ .compatible = "innolux,wj026da-03a" },
	{ .compatible = "pico,a8110-nt57900" },
	{ }
};
MODULE_DEVICE_TABLE(of, wj026da_of_match);

static struct mipi_dsi_driver wj026da_driver = {
	.probe = wj026da_probe,
	.remove = wj026da_remove,
	.shutdown = wj026da_shutdown,
	.driver = {
		.name = "panel-innolux-wj026da",
		.of_match_table = wj026da_of_match,
	},
};
module_mipi_dsi_driver(wj026da_driver);

MODULE_DESCRIPTION("Innolux WJ026DA-03A panel driver (NT57900 DDIC)");
MODULE_LICENSE("GPL");
