/*
 * This file is part of wlcore
 *
 * Copyright (C) 2011 Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/module.h>

#include "debug.h"
#include "wlcore.h"
#include "init.h"
#include "cmd.h"
#include "acx.h"

static int wlcore_init_templates(struct wlcore *wl)
{
	int ret, i;

	/*
	 * The firmware cannot allocate memory for the templates
	 * dynamically, so we send empty templates to reserve memory
	 * during initialization.
	 */

	ret = wlcore_cmd_template_set(wl, TEMPL_CFG_PROBE_REQ_2_4, NULL,
				      WLCORE_TEMPL_DFLT_SIZE,
				      0, WLCORE_RATE_AUTOMATIC);
	if (ret < 0)
		return ret;

	ret = wlcore_cmd_template_set(wl, TEMPL_CFG_PROBE_REQ_5, NULL,
				      WLCORE_TEMPL_DFLT_SIZE,
				      0, WLCORE_RATE_AUTOMATIC);
	if (ret < 0)
		return ret;

	ret = wlcore_cmd_template_set(wl, TEMPL_NULL_DATA, NULL,
				      sizeof(struct ieee80211_hdr_3addr),
				      0, WLCORE_RATE_AUTOMATIC);
	if (ret < 0)
		return ret;

	ret = wlcore_cmd_template_set(wl, TEMPL_PS_POLL, NULL,
				      sizeof(struct ieee80211_pspoll),
				      0, WLCORE_RATE_AUTOMATIC);
	if (ret < 0)
		return ret;

	ret = wlcore_cmd_template_set(wl, TEMPL_QOS_NULL_DATA, NULL,
				      sizeof (struct ieee80211_qos_hdr),
				      0, WLCORE_RATE_AUTOMATIC);
	if (ret < 0)
		return ret;

	ret = wlcore_cmd_template_set(wl, TEMPL_PROBE_RESPONSE, NULL,
				      WLCORE_TEMPL_DFLT_SIZE,
				      0, WLCORE_RATE_AUTOMATIC);
	if (ret < 0)
		return ret;

	ret = wlcore_cmd_template_set(wl, TEMPL_BEACON, NULL,
				      WLCORE_TEMPL_DFLT_SIZE,
				      0, WLCORE_RATE_AUTOMATIC);
	if (ret < 0)
		return ret;

	ret = wlcore_cmd_template_set(wl, TEMPL_ARP_RSP, NULL,
				      sizeof (struct wlcore_arp_rsp_tmpl),
				      0, WLCORE_RATE_AUTOMATIC);
	if (ret < 0)
		return ret;

	/*
	 * Put very large empty placeholders for all templates. These
	 * reserve memory for later.
	 */
	ret = wlcore_cmd_template_set(wl, TEMPL_AP_PROBE_RESPONSE, NULL,
				      WLCORE_TEMPL_MAX_SIZE,
				      0, WLCORE_RATE_AUTOMATIC);
	if (ret < 0)
		return ret;

	ret = wlcore_cmd_template_set(wl, TEMPL_AP_BEACON, NULL,
				      WLCORE_TEMPL_MAX_SIZE,
				      0, WLCORE_RATE_AUTOMATIC);
	if (ret < 0)
		return ret;

	ret = wlcore_cmd_template_set(wl, TEMPL_DEAUTH_AP, NULL,
				      sizeof (struct wlcore_deauth_tmpl),
				      0, WLCORE_RATE_AUTOMATIC);
	if (ret < 0)
		return ret;

	for (i = 0; i < TEMPL_KLV_IDX_MAX; i++) {
		ret = wlcore_cmd_template_set(wl, TEMPL_KLV, NULL,
					      WLCORE_TEMPL_DFLT_SIZE,
					      i, WLCORE_RATE_AUTOMATIC);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int wlcore_read_mem_map(struct wlcore *wl)
{
	int ret;

	wl->mem_map = kzalloc(sizeof(*wl->mem_map),
				     GFP_KERNEL);
	if (!wl->mem_map) {
		wlcore_error("couldn't allocate memory map");
		ret = -ENOMEM;
		goto out;
	}

	ret = wlcore_acx_get_mem_map(wl, wl->mem_map);
	if (ret < 0)
		goto out_free;

	goto out;

out_free:
	kfree(wl->mem_map);
out:
	return ret;
}

int wlcore_hw_init(struct wlcore *wl)
{
	int ret = 0;

	/* TODO: do we need an op to set the general/radio parameters? */

	/* TODO: can we group the RX init and TX init calls? */

	ret = wl->ops->cfg_host_if(wl);
	if (ret < 0)
		goto out;

	ret = wlcore_init_templates(wl);
	if (ret < 0)
		goto out;

	ret = wlcore_acx_mem_cfg(wl);
	if (ret < 0)
		goto out;

	/* TODO: configure the FW logger */

	/* TODO: configure BT coexistence */

	ret = wlcore_read_mem_map(wl);
	if (ret < 0)
		goto out;

	/* TODO: add cs config op */

	ret = wlcore_acx_rx_msdu_lifetime(wl);
	if (ret < 0)
		goto out_free_memmap;

	/* TODO: add dco_itrim config op */

	ret = wlcore_acx_tx_config(wl);
	if (ret < 0)
		goto out_free_memmap;

	ret = wlcore_acx_rx_irq_config(wl);
	if (ret < 0)
		goto out_free_memmap;

	ret = wlcore_acx_cca_threshold(wl);
	if (ret < 0)
		goto out_free_memmap;

	ret = wlcore_acx_frag_threshold(wl, wl->hw->wiphy->frag_threshold);
	if (ret < 0)
		goto out_free_memmap;

	ret = wlcore_cmd_enable_rx_tx(wl);
	if (ret < 0)
		goto out_free_memmap;

	/* configure PM */
	ret = wlcore_acx_pm_config(wl);
	if (ret < 0)
		goto out_free_memmap;

	ret = wlcore_acx_set_rate_mgmt_params(wl);
	if (ret < 0)
		goto out_free_memmap;

	/* configure hangover */
	ret = wlcore_acx_config_hangover(wl);
	if (ret < 0)
		goto out_free_memmap;

out_free_memmap:
	kfree(wl->mem_map);
out:
	return ret;
}

void wlcore_hw_deinit(struct wlcore *wl)
{
	kfree(wl->mem_map);
}
