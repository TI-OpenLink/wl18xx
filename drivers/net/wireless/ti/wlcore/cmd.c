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

	wlcore_debug(DEBUG_CMD, "cmd configure (%d)", id);

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
