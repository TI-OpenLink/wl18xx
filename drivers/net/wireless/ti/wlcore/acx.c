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

#include "wlcore.h"
#include "acx.h"
#include "debug.h"

int wlcore_acx_event_mbox_mask(struct wlcore *wl, u32 event_mask)
{
	struct acx_event_mask *mask;
	int ret;

	wlcore_debug(DEBUG_ACX, "acx event mbox mask");

	mask = kzalloc(sizeof(*mask), GFP_KERNEL);
	if (!mask) {
		ret = -ENOMEM;
		goto out;
	}

	/* high event mask is unused */
	mask->high_event_mask = cpu_to_le32(0xFFFFFFFF);
	mask->event_mask = cpu_to_le32(event_mask);

	ret = wlcore_cmd_configure(wl, ACX_EVENT_MBOX_MASK,
				   mask, sizeof(*mask));
	if (ret < 0) {
		wlcore_warning("failed to set acx_event_mbox_mask: %d", ret);
		goto out;
	}

out:
	kfree(mask);
	return ret;
}

int wlcore_acx_mem_cfg(struct wlcore *wl)
{
	struct acx_config_memory *mem_conf;
	struct wlcore_conf_hw_mem *mem = &wl->conf->hw_mem;
	int ret;

	wlcore_debug(DEBUG_ACX, "wlcore mem cfg");

	mem_conf = kzalloc(sizeof(*mem_conf), GFP_KERNEL);
	if (!mem_conf) {
		ret = -ENOMEM;
		goto out;
	}

	/* memory config */
	mem_conf->num_stations = mem->num_stations;
	mem_conf->rx_mem_block_num = mem->rx_block_num;
	mem_conf->tx_min_mem_block_num = mem->tx_min_block_num;
	mem_conf->num_ssid_profiles = mem->ssid_profiles;
	mem_conf->total_tx_descriptors =
		cpu_to_le32(mem->num_tx_descriptors);
	mem_conf->dyn_mem_enable = mem->dynamic_memory;
	mem_conf->tx_free_req = mem->min_req_tx_blocks;
	mem_conf->rx_free_req = mem->min_req_rx_blocks;
	mem_conf->tx_min = mem->tx_min;
	/* TODO: add correct number of fwlog mem_blocks */
	mem_conf->fwlog_blocks = 0;

	ret = wlcore_cmd_configure(wl, ACX_MEM_CFG, mem_conf,
				   sizeof(*mem_conf));
	if (ret < 0) {
		wlcore_warning("wlcore mem config failed: %d", ret);
		goto out;
	}
out:
	kfree(mem_conf);
	return ret;
}

int wlcore_acx_get_mem_map(struct wlcore *wl,
			   struct wlcore_acx_mem_map *map)
{
	int ret;

	wlcore_debug(DEBUG_ACX, "acx get mem map");

	ret = wlcore_cmd_interrogate(wl, ACX_MEM_MAP, wl->mem_map,
				     sizeof(*wl->mem_map));
	if (ret < 0) {
		wlcore_error("couldn't retrieve firmware memory map");
		goto out;
	}

	/* initialize TX block book keeping */
	wl->tx_blocks_available = le32_to_cpu(wl->mem_map->num_tx_mem_blocks);
	wlcore_debug(DEBUG_TX, "available tx blocks: %d",
		     wl->tx_blocks_available);

out:
	return ret;
}

int wlcore_acx_tx_config(struct wlcore *wl)
{
	struct acx_tx_config_options *acx;
	int ret = 0;

	wlcore_debug(DEBUG_ACX, "acx tx config");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);

	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->complete_timeout = cpu_to_le16(wl->conf->tx.complete_timeout);
	acx->complete_threshold = cpu_to_le16(wl->conf->tx.complete_threshold);
	ret = wlcore_cmd_configure(wl, ACX_TX_CONFIG_OPT, acx, sizeof(*acx));
	if (ret < 0) {
		wlcore_warning("failed to set tx config: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int wlcore_acx_cca_threshold(struct wlcore *wl)
{
	struct acx_cca_threshold *acx;
	int ret;

	wlcore_debug(DEBUG_ACX, "acx cca threshold");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->rx_cca_threshold = 0; /* deprecated? */
	acx->tx_energy_detection = wl->conf->tx.energy_detection;

	ret = wlcore_cmd_configure(wl, ACX_CCA_THRESHOLD,
				   acx, sizeof(*acx));
	if (ret < 0)
		wlcore_warning("failed to set cca threshold: %d", ret);

out:
	kfree(acx);
	return ret;
}

int wlcore_acx_frag_threshold(struct wlcore *wl, u32 frag_threshold)
{
	struct acx_frag_threshold *acx;
	int ret = 0;

	/* if fragmentation is not configured or out of range, use the max */
	if (frag_threshold > IEEE80211_MAX_FRAG_THRESHOLD)
		frag_threshold = IEEE80211_MAX_FRAG_THRESHOLD;

	wlcore_debug(DEBUG_ACX, "acx frag threshold: %d", frag_threshold);

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);

	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->frag_threshold = cpu_to_le16((u16)frag_threshold);
	ret = wlcore_cmd_configure(wl, ACX_FRAG_CFG, acx, sizeof(*acx));
	if (ret < 0) {
		wlcore_warning("Setting of frag threshold failed: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int wlcore_acx_rx_msdu_lifetime(struct wlcore *wl)
{
	struct acx_rx_msdu_lifetime *acx;
	int ret;

	wlcore_debug(DEBUG_ACX, "acx rx msdu lifetime");

	acx = kzalloc(sizeof(*acx), GFP_KERNEL);
	if (!acx) {
		ret = -ENOMEM;
		goto out;
	}

	acx->lifetime = cpu_to_le32(wl->conf->rx.msdu_lifetime);
	ret = wlcore_cmd_configure(wl, ACX_DOT11_RX_MSDU_LIFE_TIME,
				   acx, sizeof(*acx));
	if (ret < 0) {
		wlcore_warning("failed to set rx msdu lifetime: %d", ret);
		goto out;
	}

out:
	kfree(acx);
	return ret;
}

int wlcore_acx_rx_irq_config(struct wlcore *wl)
{
	struct acx_rx_irq_config *rx_conf;
	int ret;

	wlcore_debug(DEBUG_ACX, "wlcore rx irq config");

	rx_conf = kzalloc(sizeof(*rx_conf), GFP_KERNEL);
	if (!rx_conf) {
		ret = -ENOMEM;
		goto out;
	}

	rx_conf->threshold = cpu_to_le16(wl->conf->rx.irq_pkt_threshold);
	rx_conf->timeout = cpu_to_le16(wl->conf->rx.irq_timeout);
	rx_conf->mblk_threshold = cpu_to_le16(wl->conf->rx.irq_blk_threshold);
	rx_conf->queue_type = wl->conf->rx.queue_type;

	ret = wlcore_cmd_configure(wl, ACX_RX_CONFIG_OPT, rx_conf,
				   sizeof(*rx_conf));
	if (ret < 0) {
		wlcore_warning("wlcore rx irq config failed: %d", ret);
		goto out;
	}

out:
	kfree(rx_conf);
	return ret;
}
