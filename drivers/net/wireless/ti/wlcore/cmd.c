/*
 * This file is part of wlcore
 *
 * Copyright (C) 2009-2010 Nokia Corporation
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

#include <linux/export.h>

#include "wlcore.h"
#include "io.h"
#include "cmd.h"
#include "acx.h"

/*
 * send command to firmware
 *
 * @wl: wl struct
 * @id: command id
 * @buf: buffer containing the command, must work with dma
 * @len: length of the buffer
 * @res_len: length of the response to be read
 */
int wlcore_cmd_send(struct wlcore *wl, u16 id, void *buf, size_t len,
		    size_t res_len)
{
	struct wlcore_cmd_header *cmd;
	unsigned long timeout;
	u32 intr;
	int ret = 0;
	u16 status;
	u16 poll_count = 0;

	cmd = buf;
	cmd->id = cpu_to_le16(id);
	cmd->status = 0;

	BUG_ON(len % 4 != 0);

	wlcore_write(wl, wl->cmd_box_addr, buf, len, false);

	wlcore_write_trigger(wl, TRIG_CMD);

	timeout = jiffies + msecs_to_jiffies(WLCORE_COMMAND_TIMEOUT);

	intr = wlcore_read_reg(wl, REG_INTERRUPT_NO_CLEAR);
	while (!(intr & INTR_CMD_COMPLETE)) {
		if (WARN(time_after(jiffies, timeout),
			 "command complete timeout")) {
			ret = -ETIMEDOUT;
			goto out;
		}

		poll_count++;
		if (poll_count < WLCORE_CMD_FAST_POLL_COUNT)
			udelay(10);
		else
			msleep(1);

		intr = wlcore_read_reg(wl, REG_INTERRUPT_NO_CLEAR);
	}

	/* read back the status code of the command */
	if (res_len == 0)
		res_len = sizeof(struct wlcore_cmd_header);
	wlcore_read(wl, wl->cmd_box_addr, cmd, res_len, false);

	status = le16_to_cpu(cmd->status);
	if (WARN((status != CMD_STATUS_SUCCESS), "command failed %d", status)) {
		ret = -EIO;
		goto out;
	}

	wlcore_write_reg(wl, REG_INTERRUPT_ACK, INTR_CMD_COMPLETE);

	goto out;

out:
	return ret;
}

/**
 * write acx value to firmware
 *
 * @wl: wl struct
 * @id: acx id
 * @buf: buffer containing acx, including all headers, must work with dma
 * @len: length of buf
 */
int wlcore_cmd_configure(struct wlcore *wl, u16 id, void *buf, size_t len)
{
	struct acx_header *acx = buf;
	int ret;

	wlcore_debug(DEBUG_CMD, "cmd configure (0x%04X)", id);

	acx->id = cpu_to_le16(id);

	/* payload length, does not include any headers */
	acx->len = cpu_to_le16(len - sizeof(*acx));

	ret = wlcore_cmd_send(wl, CMD_CONFIGURE, acx, len, 0);
	if (ret < 0) {
		wlcore_warning("CONFIGURE command failed");
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(wlcore_cmd_configure);

/**
 * read acx from firmware
 *
 * @wl: wl struct
 * @id: acx id
 * @buf: buffer for the response, including all headers, must work with dma
 * @len: length of buf
 */
int wlcore_cmd_interrogate(struct wlcore *wl, u16 id, void *buf, size_t len)
{
	struct acx_header *acx = buf;
	int ret;

	wlcore_debug(DEBUG_CMD, "cmd interrogate");

	acx->id = cpu_to_le16(id);

	/* payload length, does not include any headers */
	acx->len = cpu_to_le16(len - sizeof(*acx));

	ret = wlcore_cmd_send(wl, CMD_INTERROGATE, acx, sizeof(*acx), len);
	if (ret < 0)
		wlcore_error("INTERROGATE command failed");

	return ret;
}

int wlcore_cmd_template_set(struct wlcore *wl, u16 template_id, void *buf,
			    size_t buf_len, int index, u32 rates)
{
	struct wlcore_cmd_template_set *cmd;
	int ret = 0;

	wlcore_debug(DEBUG_CMD, "cmd template_set %d", template_id);

	WARN_ON(buf_len > WLCORE_TEMPL_MAX_SIZE);
	buf_len = min_t(size_t, buf_len, WLCORE_TEMPL_MAX_SIZE);

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	cmd->len = cpu_to_le16(buf_len);
	cmd->template_type = template_id;
	cmd->enabled_rates = cpu_to_le32(rates);
	cmd->short_retry_limit = wl->conf->tx.tmpl_short_retry_limit;
	cmd->long_retry_limit = wl->conf->tx.tmpl_long_retry_limit;
	cmd->index = index;

	if (buf)
		memcpy(cmd->template_data, buf, buf_len);

	ret = wlcore_cmd_send(wl, CMD_SET_TEMPLATE, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		wlcore_warning("cmd set_template failed: %d", ret);
		goto out_free;
	}

out_free:
	kfree(cmd);

out:
	return ret;
}

int wlcore_cmd_enable_rx_tx(struct wlcore *wl)
{
	struct wlcore_cmd_enable_disable_rx_tx *cmd;
	int ret;

	wlcore_debug(DEBUG_CMD, "cmd enable rx and tx");

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	cmd->channel = 0; /* deprecated */

	ret = wlcore_cmd_send(wl, CMD_ENABLE_RX, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		wlcore_warning("cmd enable rx failed: %d", ret);
		goto out_free;
	}

	ret = wlcore_cmd_send(wl, CMD_ENABLE_TX, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		wlcore_warning("cmd enable tx failed: %d", ret);
		goto out_free;
	}

out_free:
	kfree(cmd);
out:
	return ret;
}

int wlcore_cmd_role_enable(struct wlcore *wl, u8 *addr, u8 role_type,
			   u8 *role_id)
{
	struct wlcore_cmd_role_enable *cmd;
	int ret;

	wlcore_debug(DEBUG_CMD, "cmd role enable");

	if (WARN_ON(*role_id != WLCORE_INVALID_ROLE_ID))
		return -EBUSY;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	/* get role id */
	cmd->role_id = find_first_zero_bit(wl->roles_map, WLCORE_MAX_ROLES);
	if (cmd->role_id >= WLCORE_MAX_ROLES) {
		ret = -EBUSY;
		goto out_free;
	}

	memcpy(cmd->mac_address, addr, ETH_ALEN);
	cmd->role_type = role_type;

	ret = wlcore_cmd_send(wl, CMD_ROLE_ENABLE, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		wlcore_error("failed to initiate cmd role enable");
		goto out_free;
	}

	__set_bit(cmd->role_id, wl->roles_map);
	*role_id = cmd->role_id;

out_free:
	kfree(cmd);

out:
	return ret;
}

int wlcore_cmd_role_disable(struct wlcore *wl, u8 *role_id)
{
	struct wlcore_cmd_role_disable *cmd;
	int ret;

	wlcore_debug(DEBUG_CMD, "cmd role disable");

	if (WARN_ON(*role_id == WLCORE_INVALID_ROLE_ID))
		return -ENOENT;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}
	cmd->role_id = *role_id;

	ret = wlcore_cmd_send(wl, CMD_ROLE_DISABLE, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		wlcore_error("failed to initiate cmd role disable");
		goto out_free;
	}

	__clear_bit(*role_id, wl->roles_map);
	*role_id = WLCORE_INVALID_ROLE_ID;

out_free:
	kfree(cmd);

out:
	return ret;
}

int wlcore_allocate_link(struct wlcore *wl, struct wlcore_vif *wlvif, u8 *hlid)
{
	u8 link = find_first_zero_bit(wl->links_map, WLCORE_MAX_LINKS);
	if (link >= WLCORE_MAX_LINKS)
		return -EBUSY;

	__set_bit(link, wl->links_map);
	__set_bit(link, wlvif->links_map);
	*hlid = link;
	return 0;
}

void wlcore_free_link(struct wlcore *wl, struct wlcore_vif *wlvif, u8 *hlid)
{
	if (*hlid == WLCORE_INVALID_LINK_ID)
		return;

	__clear_bit(*hlid, wl->links_map);
	__clear_bit(*hlid, wlvif->links_map);
	*hlid = WLCORE_INVALID_LINK_ID;
}

static int wlcore_cmd_role_start_dev(struct wlcore *wl,
				     struct wlcore_vif *wlvif)
{
	struct wlcore_cmd_role_start *cmd;
	int ret;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	wlcore_debug(DEBUG_CMD, "cmd role start dev %d", wlvif->dev_role_id);

	cmd->role_id = wlvif->dev_role_id;
	if (wlvif->band == IEEE80211_BAND_5GHZ)
		cmd->band = WLCORE_BAND_5GHZ;
	cmd->channel = wlvif->channel;

	if (wlvif->dev_hlid == WLCORE_INVALID_LINK_ID) {
		ret = wlcore_allocate_link(wl, wlvif, &wlvif->dev_hlid);
		if (ret)
			goto out_free;
	}
	cmd->device.hlid = wlvif->dev_hlid;
	cmd->device.session = wlvif->session_counter;

	wlcore_debug(DEBUG_CMD, "role start: roleid=%d, hlid=%d, session=%d",
		     cmd->role_id, cmd->device.hlid, cmd->device.session);

	ret = wlcore_cmd_send(wl, CMD_ROLE_START, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		wlcore_error("failed to initiate cmd role enable");
		goto err_hlid;
	}

	goto out_free;

err_hlid:
	/* clear links on error */
	wlcore_free_link(wl, wlvif, &wlvif->dev_hlid);

out_free:
	kfree(cmd);

out:
	return ret;
}

static int wlcore_cmd_role_stop_dev(struct wlcore *wl,
				    struct wlcore_vif *wlvif)
{
	struct wlcore_cmd_role_stop *cmd;
	int ret;

	if (WARN_ON(wlvif->dev_hlid == WLCORE_INVALID_LINK_ID))
		return -EINVAL;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	wlcore_debug(DEBUG_CMD, "cmd role stop dev");

	cmd->role_id = wlvif->dev_role_id;
	cmd->disc_type = DISCONNECT_IMMEDIATE;
	cmd->reason = cpu_to_le16(WLAN_REASON_UNSPECIFIED);

	ret = wlcore_cmd_send(wl, CMD_ROLE_STOP, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		wlcore_error("failed to initiate cmd role stop");
		goto out_free;
	}

	ret = wlcore_event_wait(wl, EVENT_DISCONNECT_COMPLETE);
	if (ret < 0) {
		wlcore_error("cmd role stop dev event completion error");
		goto out_free;
	}

	wlcore_free_link(wl, wlvif, &wlvif->dev_hlid);

out_free:
	kfree(cmd);

out:
	return ret;
}

static int wlcore_cmd_roc(struct wlcore *wl, struct wlcore_vif *wlvif,
			  u8 role_id)
{
	struct wlcore_cmd_roc *cmd;
	int ret = 0;

	wlcore_debug(DEBUG_CMD, "cmd roc %d (%d)", wlvif->channel, role_id);

	if (WARN_ON(role_id == WLCORE_INVALID_ROLE_ID))
		return -EINVAL;

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}

	cmd->role_id = role_id;
	cmd->channel = wlvif->channel;
	if (wlvif->band == IEEE80211_BAND_5GHZ)
		cmd->band = WLCORE_BAND_5GHZ;

	ret = wlcore_cmd_send(wl, CMD_REMAIN_ON_CHANNEL, cmd, sizeof(*cmd), 0);
	if (ret < 0) {
		wlcore_error("failed to send ROC command");
		goto out_free;
	}

out_free:
	kfree(cmd);

out:
	return ret;
}

static int wlcore_cmd_croc(struct wlcore *wl, u8 role_id)
{
	struct wlcore_cmd_croc *cmd;
	int ret = 0;

	wlcore_debug(DEBUG_CMD, "cmd croc (%d)", role_id);

	cmd = kzalloc(sizeof(*cmd), GFP_KERNEL);
	if (!cmd) {
		ret = -ENOMEM;
		goto out;
	}
	cmd->role_id = role_id;

	ret = wlcore_cmd_send(wl, CMD_CANCEL_REMAIN_ON_CHANNEL, cmd,
			      sizeof(*cmd), 0);
	if (ret < 0) {
		wlcore_error("failed to send ROC command");
		goto out_free;
	}

out_free:
	kfree(cmd);

out:
	return ret;
}

int wlcore_roc(struct wlcore *wl, struct wlcore_vif *wlvif, u8 role_id)
{
	int ret = 0;

	if (WARN_ON(test_bit(role_id, wl->roc_map)))
		return 0;

	ret = wlcore_cmd_roc(wl, wlvif, role_id);
	if (ret < 0)
		goto out;

	ret = wlcore_event_wait(wl, EVENT_REMAIN_ON_CHANNEL_COMPLETE);
	if (ret < 0) {
		wlcore_error("cmd roc event completion error");
		goto out;
	}

	__set_bit(role_id, wl->roc_map);
out:
	return ret;
}

int wlcore_croc(struct wlcore *wl, u8 role_id)
{
	int ret = 0;

	if (WARN_ON(!test_bit(role_id, wl->roc_map)))
		return 0;

	ret = wlcore_cmd_croc(wl, role_id);
	if (ret < 0)
		goto out;

	__clear_bit(role_id, wl->roc_map);
out:
	return ret;
}

/* start dev role and roc on its channel */
int wlcore_start_dev(struct wlcore *wl, struct wlcore_vif *wlvif)
{
	int ret;

	if (WARN_ON(!(wlvif->bss_type == BSS_TYPE_STA_BSS ||
		      wlvif->bss_type == BSS_TYPE_IBSS)))
		return -EINVAL;

	ret = wlcore_cmd_role_start_dev(wl, wlvif);
	if (ret < 0)
		goto out;

	ret = wlcore_roc(wl, wlvif, wlvif->dev_role_id);
	if (ret < 0)
		goto out_stop;

	return 0;

out_stop:
	wlcore_cmd_role_stop_dev(wl, wlvif);
out:
	return ret;
}

/* croc dev hlid, and stop the role */
int wlcore_stop_dev(struct wlcore *wl, struct wlcore_vif *wlvif)
{
	int ret;

	if (WARN_ON(!(wlvif->bss_type == BSS_TYPE_STA_BSS ||
		      wlvif->bss_type == BSS_TYPE_IBSS)))
		return -EINVAL;

	if (test_bit(wlvif->dev_role_id, wl->roc_map)) {
		ret = wlcore_croc(wl, wlvif->dev_role_id);
		if (ret < 0)
			goto out;
	}

	ret = wlcore_cmd_role_stop_dev(wl, wlvif);
	if (ret < 0)
		goto out;
out:
	return ret;
}
