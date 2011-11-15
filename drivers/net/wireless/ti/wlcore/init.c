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

int wlcore_hw_init(struct wlcore *wl)
{
	int ret = 0;

	/* TODO: do we need an op to set the general/radio parameters? */

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

	/* TODO: add memmap reading (TX and TX and more for wl12xx) */

	/* TODO: add cs config op */

	ret = wlcore_acx_rx_msdu_lifetime(wl);
	if (ret < 0)
		goto out;

	/* TODO: add dco_itrim config op */

	/* TODO: add tx config -- can it be after rx_irq_config? */

	ret = wlcore_acx_rx_irq_config(wl);
	if (ret < 0)
		goto out;

	/* TODO: add CCA (part of TX) */

	/* TODO: add fragmentation threshold (part of TX) */

	ret = wlcore_cmd_enable_rx(wl);
	if (ret < 0)
		goto out;

out:
	return ret;
}
