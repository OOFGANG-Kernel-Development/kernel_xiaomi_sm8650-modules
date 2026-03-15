/* Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 * Copyright (c) 2022 Qualcomm Innovation Center, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/ratelimit.h>
#include "cam_mem_mgr_api.h"
#include "cam_req_mgr_workq.h"
#include "cam_smmu_api.h"
#include "ais_vfe_soc.h"
#include "ais_vfe_core.h"
#include "cam_debug_util.h"
#include "ais_isp_trace.h"

#define CAM_VFE_BUS_VER3_IRQ_REG0                0
#define CAM_VFE_BUS_VER3_IRQ_REG1                1
#define CAM_VFE_BUS_VER3_IRQ_MAX                 2

/*Allow max of 4 HW FIFO Q + 2 delayed buffers before error*/
#define MAX_NUM_BUF_SW_FIFOQ_ERR 6

int ais_vfe_process_buf_done(struct cam_hw_info *vfe_hw,
	struct ais_ife_proc_buf_done_args *buf_done);

static ssize_t ais_vfe_debug_reg_read(struct file *file,
	char __user *user_buf, size_t count, loff_t *ppos)
{
	char    *buf;
	ssize_t  rc;
	struct ais_vfe_hw_core_info *core_info =
		(struct ais_vfe_hw_core_info *)file->private_data;

	if (!core_info)
		return -EINVAL;

	buf = kzalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	core_info->reg_val = cam_io_r_mb(core_info->mem_base + core_info->offset);
	rc = snprintf(buf, count, "0x%x\n", core_info->reg_val);

	rc = simple_read_from_buffer(user_buf, count, ppos, buf, rc);

	kfree(buf);
	return rc;
}

static ssize_t ais_vfe_debug_reg_write(struct file *file,
	const char __user *user_buf, size_t count, loff_t *ppos)
{
	char    buf[128];
	char   *start = buf;
	size_t  buf_size;
	struct ais_vfe_hw_core_info *core_info =
		(struct ais_vfe_hw_core_info *)file->private_data;

	if (!core_info)
		return -EINVAL;

	buf_size = min(count, (sizeof(buf)-1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;
	buf[buf_size] = 0;

	core_info->reg_val = simple_strtoul(start, &start, 16);
	cam_io_w_mb(core_info->reg_val, core_info->mem_base + core_info->offset);
	CAM_DBG(CAM_ISP, "register value = 0x%x", core_info->reg_val);

	return buf_size;
}

static const struct file_operations ais_vfe_debug_reg = {
	.open = simple_open,
	.read = ais_vfe_debug_reg_read,
	.write = ais_vfe_debug_reg_write,
	.llseek = default_llseek,
};

static int ais_vfe_create_debugfs_entry(struct ais_vfe_hw_core_info *core_info)
{
	int rc = 0;
	struct dentry *dbgfileptr     = NULL;
	struct dentry *debugfs_device = NULL;
	char debugfs_name[DEBUGFS_NAME_MAX_SIZE];

	CAM_DBG(CAM_ISP, "create ais vfe%d debugfs.", core_info->vfe_idx);

	dbgfileptr = debugfs_lookup("camera", NULL);
	if (!dbgfileptr) {
		CAM_ERR(CAM_ISP, "vfe%d: camera root debugfs dir lookup failed", core_info->vfe_idx);
		rc = -ENOENT;
		goto end;
	}

	dbgfileptr = debugfs_lookup("ais_vfe", dbgfileptr);
	if (!dbgfileptr) {
		CAM_ERR(CAM_ISP, "vfe%d: ais-vfe root debugfs dir lookup failed",
				core_info->vfe_idx);
		rc = -ENOENT;
		goto end;
	}

	snprintf(debugfs_name, DEBUGFS_NAME_MAX_SIZE, "ais_vfe%u",
			core_info->vfe_idx);
	dbgfileptr = debugfs_create_dir(debugfs_name, dbgfileptr);
	if (!dbgfileptr) {
		CAM_ERR(CAM_ISP, "debugfs directory %s creation fail", debugfs_name);
		rc = -ENOENT;
		return 0;
	}

	debugfs_device = dbgfileptr;
	core_info->entry = debugfs_device;

	debugfs_create_u32("offset", 0600, debugfs_device, &core_info->offset);
	dbgfileptr = debugfs_create_file("register_val", 0600,
			debugfs_device, core_info, &ais_vfe_debug_reg);

end:
	return rc;
}

static void ais_vfe_remove_debugfs_entry(struct ais_vfe_hw_core_info *core_info)
{
	debugfs_remove_recursive(core_info->entry);
}

static void ais_clear_rdi_path(struct ais_vfe_rdi_output *rdi_path)
{
	int i;

	rdi_path->frame_cnt = 0;

	rdi_path->num_buffer_hw_q = 0;
	INIT_LIST_HEAD(&rdi_path->buffer_q);
	INIT_LIST_HEAD(&rdi_path->buffer_hw_q);
	INIT_LIST_HEAD(&rdi_path->free_buffer_list);
	for (i = 0; i < AIS_VFE_MAX_BUF; i++) {
		INIT_LIST_HEAD(&rdi_path->buffers[i].list);
		list_add_tail(&rdi_path->buffers[i].list,
				&rdi_path->free_buffer_list);
	}

	memset(&rdi_path->last_sof_info, 0, sizeof(rdi_path->last_sof_info));

	rdi_path->num_sof_info_q = 0;
	INIT_LIST_HEAD(&rdi_path->sof_info_q);
	INIT_LIST_HEAD(&rdi_path->free_sof_info_list);
	for (i = 0; i < AIS_VFE_MAX_SOF_INFO; i++) {
		INIT_LIST_HEAD(&rdi_path->sof_info[i].list);
		list_add_tail(&rdi_path->sof_info[i].list,
				&rdi_path->free_sof_info_list);
	}
}

static int ais_vfe_top_hw_init(struct ais_vfe_hw_core_info *core_info)
{
	struct cam_vfe_top_ver4_hw_info  *top_hw_info = NULL;
	uint32_t                          hw_version;

	top_hw_info = core_info->vfe_hw_info->top_hw_info;
	hw_version = cam_io_r_mb(core_info->mem_base + top_hw_info->common_reg->hw_version);
	CAM_DBG(CAM_ISP, "VFE:%d hw-version:0x%x", core_info->vfe_idx, hw_version);

	return 0;
}

int ais_vfe_get_hw_caps(void *hw_priv, void *get_hw_cap_args, uint32_t arg_size)
{
	struct cam_hw_info                *vfe_dev = hw_priv;
	struct ais_vfe_hw_core_info       *core_info = NULL;
	int rc = 0;

	CAM_DBG(CAM_ISP, "Enter");
	if (!hw_priv) {
		CAM_ERR(CAM_ISP, "Invalid arguments");
		return -EINVAL;
	}

	core_info = (struct ais_vfe_hw_core_info *)vfe_dev->core_info;

	CAM_WARN(CAM_ISP, "VFE%d get_hw_caps not implemented",
			core_info->vfe_idx);

	rc = -EPERM;

	CAM_DBG(CAM_ISP, "Exit");
	return rc;
}

int ais_vfe_init_hw(void *hw_priv, void *init_hw_args, uint32_t arg_size)
{
	struct cam_hw_info                *vfe_hw = hw_priv;
	struct cam_hw_soc_info            *soc_info = NULL;
	struct ais_vfe_hw_core_info       *core_info = NULL;
	int rc = 0;
	uint32_t                           reset_core_args =
		AIS_VFE_HW_RESET_HW_AND_REG;

	CAM_DBG(CAM_ISP, "Enter");
	if (!hw_priv) {
		CAM_ERR(CAM_ISP, "Invalid arguments");
		return -EINVAL;
	}

	mutex_lock(&vfe_hw->hw_mutex);
	vfe_hw->open_count++;
	if (vfe_hw->open_count > 1) {
		mutex_unlock(&vfe_hw->hw_mutex);
		CAM_DBG(CAM_ISP, "VFE has already been initialized cnt %d",
				vfe_hw->open_count);
		return 0;
	}
	mutex_unlock(&vfe_hw->hw_mutex);

	soc_info = &vfe_hw->soc_info;
	core_info = (struct ais_vfe_hw_core_info *)vfe_hw->core_info;

	/* Turn ON Regulators, Clocks and other SOC resources */
	rc = ais_vfe_enable_soc_resources(soc_info);
	if (rc) {
		CAM_ERR(CAM_ISP, "Enable SOC failed");
		rc = -EFAULT;
		goto decrement_open_cnt;
	}

	CAM_DBG(CAM_ISP, "Enable soc done");

	/* Do HW Reset */
	rc = ais_vfe_reset(hw_priv, &reset_core_args, sizeof(uint32_t));
	if (rc) {
		CAM_ERR(CAM_ISP, "Reset Failed rc=%d", rc);
		goto disable_soc;
	}

	ais_vfe_top_hw_init(core_info);

	vfe_hw->hw_state = CAM_HW_STATE_POWER_UP;
	return rc;

disable_soc:
	ais_vfe_disable_soc_resources(soc_info);
decrement_open_cnt:
	mutex_lock(&vfe_hw->hw_mutex);
	vfe_hw->open_count--;
	mutex_unlock(&vfe_hw->hw_mutex);
	return rc;
}

int ais_vfe_deinit_hw(void *hw_priv, void *deinit_hw_args, uint32_t arg_size)
{
	struct cam_hw_info                *vfe_hw = hw_priv;
	struct cam_hw_soc_info            *soc_info = NULL;
	struct ais_vfe_hw_core_info       *core_info = NULL;
	int rc = 0;
	uint32_t                           reset_core_args =
		AIS_VFE_HW_RESET_HW_AND_REG;

	CAM_DBG(CAM_ISP, "Enter");
	if (!hw_priv) {
		CAM_ERR(CAM_ISP, "Invalid arguments");
		return -EINVAL;
	}

	mutex_lock(&vfe_hw->hw_mutex);
	if (!vfe_hw->open_count) {
		mutex_unlock(&vfe_hw->hw_mutex);
		CAM_ERR(CAM_ISP, "Error. Unbalanced deinit");
		return -EFAULT;
	}
	vfe_hw->open_count--;
	if (vfe_hw->open_count) {
		mutex_unlock(&vfe_hw->hw_mutex);
		CAM_DBG(CAM_ISP, "open_cnt non-zero =%d", vfe_hw->open_count);
		return 0;
	}
	mutex_unlock(&vfe_hw->hw_mutex);

	soc_info = &vfe_hw->soc_info;
	core_info = (struct ais_vfe_hw_core_info *)vfe_hw->core_info;

	rc = ais_vfe_reset(hw_priv, &reset_core_args, sizeof(uint32_t));

	/* Turn OFF Regulators, Clocks and other SOC resources */
	CAM_DBG(CAM_ISP, "Disable SOC resource");
	rc = ais_vfe_disable_soc_resources(soc_info);
	if (rc)
		CAM_ERR(CAM_ISP, "Disable SOC failed");

	vfe_hw->hw_state = CAM_HW_STATE_POWER_DOWN;

	CAM_DBG(CAM_ISP, "Exit");
	return rc;
}

int ais_vfe_force_reset(void *hw_priv, void *reset_core_args, uint32_t arg_size)
{
	struct cam_hw_info                *vfe_hw = hw_priv;
	bool require_deinit = false;
	int rc = 0;

	mutex_lock(&vfe_hw->hw_mutex);
	if (vfe_hw->open_count) {
		vfe_hw->open_count = 1;
		require_deinit = true;

	}
	mutex_unlock(&vfe_hw->hw_mutex);

	if (require_deinit) {
		CAM_INFO(CAM_ISP, "vfe deinit HW");
		rc = ais_vfe_deinit_hw(vfe_hw, NULL, 0);
	}

	CAM_DBG(CAM_ISP, "Exit (%d)", rc);

	return rc;
}

int ais_vfe_reset(void *hw_priv, void *reset_core_args, uint32_t arg_size)
{
	struct cam_hw_info          *vfe_hw     = hw_priv;
	struct ais_vfe_hw_core_info *core_info;
	struct cam_vfe_irq_hw_info  *irq_info;
	int rc = 0;

	CAM_DBG(CAM_ISP, "Enter");


	if (!hw_priv) {
		CAM_ERR(CAM_ISP, "Invalid input arguments");
		return -EINVAL;
	}

	core_info = (struct ais_vfe_hw_core_info *)vfe_hw->core_info;
	irq_info = core_info->vfe_hw_info->irq_hw_info;

	if(!(irq_info->supported_irq & CAM_VFE_HW_IRQ_CAP_RESET))
		goto skip_reset;

	reinit_completion(&vfe_hw->hw_complete);

	CAM_DBG(CAM_ISP, "Calling RESET on VFE");

	/* Wait for Completion or Timeout of 500ms */
	rc = cam_common_wait_for_completion_timeout(
			&vfe_hw->hw_complete, 500);

	if (!rc)
		CAM_ERR(CAM_ISP, "Reset Timeout");
	else
		CAM_DBG(CAM_ISP, "Reset complete (%d)", rc);

	irq_info->reset_irq_handle = 0;

skip_reset:
	CAM_DBG(CAM_ISP, "Exit");
	return rc;
}

int ais_vfe_release(void *hw_priv, void *release_args, uint32_t arg_size)
{
	struct ais_vfe_hw_core_info       *core_info = NULL;
	struct cam_hw_info                *vfe_hw  = hw_priv;
	struct ais_vfe_rdi_output         *rdi_path = NULL;
	struct ais_ife_rdi_deinit_args    *deinit_cmd;

	int rc = 0;

	if (!hw_priv || !release_args ||
			(arg_size != sizeof(struct ais_ife_rdi_deinit_args))) {
		CAM_ERR(CAM_ISP, "Invalid input arguments");
		return -EINVAL;
	}

	deinit_cmd = (struct ais_ife_rdi_deinit_args *)release_args;

	if (deinit_cmd->path >= AIS_IFE_PATH_MAX) {
		CAM_ERR(CAM_ISP, "Invalid output path %d", deinit_cmd->path);
		return -EINVAL;
	}

	core_info = (struct ais_vfe_hw_core_info *)vfe_hw->core_info;
	rdi_path = &core_info->rdi_out[deinit_cmd->path];

	mutex_lock(&vfe_hw->hw_mutex);

	if (rdi_path->state < AIS_ISP_RESOURCE_STATE_INIT_HW) {
		CAM_ERR(CAM_ISP, "RDI%d invalid state %d", deinit_cmd->path,
				rdi_path->state);
		rc = -EINVAL;
		goto EXIT;
	}

	rdi_path->state = AIS_ISP_RESOURCE_STATE_AVAILABLE;

EXIT:
	mutex_unlock(&vfe_hw->hw_mutex);

	return rc;
}

int ais_vfe_read(void *hw_priv, void *read_args, uint32_t arg_size)
{
	return -EPERM;
}

int ais_vfe_write(void *hw_priv, void *write_args, uint32_t arg_size)
{
	return -EPERM;
}

static void ais_vfe_q_bufs_to_hw(struct ais_vfe_hw_core_info *core_info,
		enum ais_ife_output_path_id path)
{
	struct ais_vfe_rdi_output *rdi_path = NULL;
	struct ais_vfe_buffer_t *vfe_buf = NULL;
	struct cam_vfe_bus_ver3_hw_info   *bus_hw_info = NULL;
	struct cam_vfe_bus_ver3_reg_offset_bus_client  *client_regs = NULL;
	uint32_t client_idx;
	uint32_t fifo_cnt = 0;
	bool is_full = false;
	struct ais_ife_rdi_get_timestamp_args get_ts;
	uint32_t iova_addr;

	rdi_path = &core_info->rdi_out[path];
	bus_hw_info = core_info->vfe_hw_info->bus_hw_info;
	client_idx = bus_hw_info->vfe_out_hw_info[path].wm_idx[0];
	client_regs = &bus_hw_info->bus_client_reg[client_idx];

	fifo_cnt = cam_io_r_mb(core_info->mem_base +
			client_regs->addr_status_2) & 0xf;
	is_full = (fifo_cnt >= AIS_VFE_FIFO_NUM_MAX);

	while (!is_full) {
		if (list_empty(&rdi_path->buffer_q))
			break;

		vfe_buf = list_first_entry(&rdi_path->buffer_q,
				struct ais_vfe_buffer_t, list);
		list_del_init(&vfe_buf->list);

		get_ts.path = path;
		get_ts.ts = &vfe_buf->ts_hw;
		core_info->csid_hw->hw_ops.process_cmd(
				core_info->csid_hw->hw_priv,
				AIS_IFE_CSID_CMD_GET_TIME_STAMP,
				&get_ts,
				sizeof(get_ts));

		if (cam_smmu_is_expanded_memory()) {
			iova_addr = CAM_36BIT_INTF_GET_IOVA_BASE(vfe_buf->iova_addr);
		} else {
			iova_addr = vfe_buf->iova_addr;
		}
		CAM_DBG(CAM_ISP, "IFE%d|RDI%d: Q %d(0x%x) FIFO:%d ts %llu",
				core_info->vfe_idx, path,
				vfe_buf->bufIdx, iova_addr,
				rdi_path->num_buffer_hw_q, vfe_buf->ts_hw.cur_sof_ts);

		cam_io_w_mb(iova_addr,
				core_info->mem_base + client_regs->image_addr);

		list_add_tail(&vfe_buf->list, &rdi_path->buffer_hw_q);
		++rdi_path->num_buffer_hw_q;

		CAM_DBG(CAM_ISP, "hw FIFO num = %d",
				cam_io_r_mb(core_info->mem_base + client_regs->addr_status_2));

		fifo_cnt = cam_io_r_mb(core_info->mem_base +
				client_regs->addr_status_2) & 0xf;
		is_full = (fifo_cnt >= AIS_VFE_FIFO_NUM_MAX);

		trace_ais_isp_vfe_enq_buf_hw(core_info->vfe_idx, path,
				vfe_buf->bufIdx, rdi_path->num_buffer_hw_q, is_full);
	}

	if (rdi_path->num_buffer_hw_q > MAX_NUM_BUF_SW_FIFOQ_ERR)
		CAM_WARN(CAM_ISP, "Excessive number of buffers in SW FIFO (%d)",
				rdi_path->num_buffer_hw_q);
}

void ais_ife_discard_old_frame_done_event(struct ais_vfe_hw_core_info *core_info,
	struct ais_ife_event_data *evt_data)
{
	uint8_t path = 0;
	uint32_t buf_idx = 0;
	struct ais_vfe_buffer_t *vfe_buf = NULL;
	struct ais_vfe_buffer_t *tmp_vfe_buf = NULL;
	struct ais_vfe_rdi_output *rdi_path = NULL;
	int rc = -1;

	if (core_info == NULL || evt_data == NULL)
		return;

	path = evt_data->msg.path;
	buf_idx = evt_data->u.frame_msg.buf_idx;

	if (path >= AIS_IFE_PATH_MAX) {
		CAM_WARN(CAM_ISP, "Invalid path:%d", path);
		return;
	}

	rdi_path = &core_info->rdi_out[path];
	if (rdi_path->state != AIS_ISP_RESOURCE_STATE_STREAMING) {
		CAM_WARN(CAM_ISP, "Not streaming state:%d", rdi_path->state);
		return;
	}

	spin_lock(&rdi_path->buffer_lock);
	if (list_empty(&rdi_path->free_buffer_list)) {
		spin_unlock(&rdi_path->buffer_lock);
		return;
	}

	list_for_each_entry_safe(vfe_buf, tmp_vfe_buf,
			&rdi_path->free_buffer_list, list) {
		if ((vfe_buf->bufIdx == buf_idx) &&
				(vfe_buf->mem_handle != 0) &&
				(vfe_buf->iova_addr != 0)) {
			list_del_init(&vfe_buf->list);
			list_add_tail(&vfe_buf->list, &rdi_path->buffer_q);

			rc = 0;
			break;
		}
	}
	spin_unlock(&rdi_path->buffer_lock);

	if (rc == 0 && vfe_buf != NULL) {
		CAM_WARN(CAM_ISP, "I%d|R%d discard old frame done buffer:%d",
				core_info->vfe_idx, path, vfe_buf->bufIdx);
	} else {
		CAM_WARN(CAM_ISP, "I%d|R%d can't find old frame done buffer:%d",
				core_info->vfe_idx, path, buf_idx);
	}
}

static int ais_vfe_cmd_enq_buf(struct ais_vfe_hw_core_info *core_info,
	struct ais_ife_enqueue_buffer_args *enq_buf)
{
	int rc;
	struct ais_vfe_buffer_t *vfe_buf[4] = {};
	struct ais_vfe_rdi_output *rdi_path = NULL;
	int32_t mmu_hdl;
	size_t  src_buf_size;
	uint32_t i = 0;
	uint32_t batch_id = 0;
	uint64_t base_addr = 0;

	if (enq_buf->path >= AIS_IFE_PATH_MAX) {
		CAM_ERR(CAM_ISP, "Invalid output path %d", enq_buf->path);
		rc = -EINVAL;
		goto EXIT;
	}

	rdi_path = &core_info->rdi_out[enq_buf->path];
	if (rdi_path->state < AIS_ISP_RESOURCE_STATE_RESERVED) {
		CAM_ERR(CAM_ISP, "RDI%d invalid state %d", enq_buf->path,
				rdi_path->state);
		rc = -EINVAL;
		goto EXIT;
	}

	spin_lock(&rdi_path->buffer_lock);
	for (batch_id = 0; batch_id < rdi_path->batchConfig.numBatchFrames; batch_id++) {
		if (!list_empty(&rdi_path->free_buffer_list)) {
			vfe_buf[batch_id] = list_first_entry(&rdi_path->free_buffer_list,
					struct ais_vfe_buffer_t, list);
			list_del_init(&vfe_buf[batch_id]->list);
		}
		if (!vfe_buf[batch_id]) {
			CAM_ERR(CAM_ISP, "RDI%d No more free buffers!", enq_buf->path);
			for (i = 0; i < batch_id; i++)
				list_add_tail(&vfe_buf[i]->list, &rdi_path->free_buffer_list);
			spin_unlock(&rdi_path->buffer_lock);
			return -ENOMEM;
		}
	}
	spin_unlock(&rdi_path->buffer_lock);


	vfe_buf[0]->mem_handle = enq_buf->buffer.mem_handle;

	mmu_hdl = core_info->iommu_hdl;

	if (cam_mem_is_secure_buf(vfe_buf[0]->mem_handle) || rdi_path->secure_mode)
		mmu_hdl = core_info->iommu_hdl_secure;

	rc = cam_mem_get_io_buf(vfe_buf[0]->mem_handle,
			mmu_hdl, &vfe_buf[0]->iova_addr, &src_buf_size, NULL, NULL);
	if (rc < 0) {
		CAM_ERR(CAM_ISP,
				"get src buf address fail mem_handle 0x%x",
				vfe_buf[0]->mem_handle);
	}
	CAM_DBG(CAM_ISP, "vfe %d buf idx %d iova_addr = 0x%x, size = %u, status = %u",
			core_info->vfe_idx, enq_buf->buffer.idx, vfe_buf[0]->iova_addr, src_buf_size, rdi_path->state);

	if (enq_buf->buffer.offset >= src_buf_size) {
		CAM_ERR(CAM_ISP, "Invalid buffer offset");
		rc = -EINVAL;
	}

	//if any error, return buffer list object to being free
	if (rc) {
		spin_lock(&rdi_path->buffer_lock);
		for (batch_id = 0; batch_id < rdi_path->batchConfig.numBatchFrames; batch_id++)
			list_add_tail(&vfe_buf[batch_id]->list, &rdi_path->free_buffer_list);
		spin_unlock(&rdi_path->buffer_lock);
	} else {
		base_addr  = vfe_buf[0]->iova_addr + enq_buf->buffer.offset;
		spin_lock(&rdi_path->buffer_lock);
		for (batch_id = 0; batch_id < rdi_path->batchConfig.numBatchFrames; batch_id++) {
			vfe_buf[batch_id]->bufIdx = enq_buf->buffer.idx;

			vfe_buf[batch_id]->iova_addr = base_addr +
				batch_id * rdi_path->batchConfig.frameIncrement;

			vfe_buf[batch_id]->batchId = batch_id;

			trace_ais_isp_vfe_enq_req(core_info->vfe_idx, enq_buf->path,
					enq_buf->buffer.idx);

			list_add_tail(&vfe_buf[batch_id]->list, &rdi_path->buffer_q);
		}
		spin_unlock(&rdi_path->buffer_lock);

		if (rdi_path->state < AIS_ISP_RESOURCE_STATE_STREAMING)
			ais_vfe_q_bufs_to_hw(core_info, enq_buf->path);
	}

EXIT:
	return rc;
}

static int ais_vfe_restore_buf_list(struct cam_hw_info *vfe_hw, int path)
{
	struct ais_vfe_hw_core_info   *core_info;
	struct ais_vfe_rdi_output *rdi_path = NULL;
	struct ais_vfe_buffer_t           *vfe_buf = NULL, *buf_tmp;

	CAM_DBG(CAM_ISP, "restore buffers for path %u", path);

	core_info = (struct ais_vfe_hw_core_info *)vfe_hw->core_info;
	rdi_path = &core_info->rdi_out[path];

	spin_lock(&rdi_path->buffer_lock);
	list_for_each_entry_safe(vfe_buf, buf_tmp, &rdi_path->buffer_hw_q, list) {
		CAM_DBG(CAM_ISP, "R%d hw buf %d addr 0x%x", path, vfe_buf->bufIdx, vfe_buf->iova_addr);
		list_del_init(&vfe_buf->list);
		--rdi_path->num_buffer_hw_q;
		list_add_tail(&vfe_buf->list, &rdi_path->buffer_q);
	}

	ais_vfe_q_bufs_to_hw(core_info, path);

	spin_unlock(&rdi_path->buffer_lock);

	return 0;
}

int ais_vfe_process_cmd(void *hw_priv, uint32_t cmd_type,
	void *cmd_args, uint32_t arg_size)
{
	struct cam_hw_info                *vfe_hw = hw_priv;
	struct cam_hw_soc_info            *soc_info = NULL;
	struct ais_vfe_hw_core_info       *core_info = NULL;
	struct cam_vfe_hw_info            *hw_info = NULL;
	int rc = 0;

	if (!hw_priv) {
		CAM_ERR(CAM_ISP, "Invalid arguments");
		return -EINVAL;
	}

	soc_info = &vfe_hw->soc_info;
	core_info = (struct ais_vfe_hw_core_info *)vfe_hw->core_info;
	hw_info = core_info->vfe_hw_info;

	switch (cmd_type) {
	case AIS_VFE_CMD_ENQ_BUFFER: {
		struct ais_ife_enqueue_buffer_args *enq_buf =
			(struct ais_ife_enqueue_buffer_args *)cmd_args;
		mutex_lock(&vfe_hw->hw_mutex);
		if (arg_size != sizeof(*enq_buf))
			rc = -EINVAL;
		else
			rc = ais_vfe_cmd_enq_buf(core_info, enq_buf);
		mutex_unlock(&vfe_hw->hw_mutex);
		break;
	}
	case AIS_VFE_CMD_PROC_BUF_DONE: {
		struct ais_ife_proc_buf_done_args *buf_done =
			(struct ais_ife_proc_buf_done_args *)cmd_args;
		if (arg_size != sizeof(*buf_done))
			rc = -EINVAL;
		else
			rc = ais_vfe_process_buf_done(vfe_hw, buf_done);
		break;
	}
	case AIS_VFE_CMD_RESTART_PATH: {
		uint32_t *path = (uint32_t *)cmd_args;
		if (arg_size != sizeof(*path))
			rc = -EINVAL;
		else
			ais_vfe_restore_buf_list(vfe_hw, *path);
		break;
	}
	default:
		CAM_ERR(CAM_ISP, "Invalid cmd type:%d", cmd_type);
		rc = -EINVAL;
		break;
	}

	return rc;
}

int ais_vfe_reserve(void *hw_priv, void *reserve_args, uint32_t arg_size)
{
	struct ais_vfe_hw_core_info       *core_info = NULL;
	struct cam_hw_info                *vfe_hw  = hw_priv;
	struct ais_vfe_rdi_output         *rdi_path = NULL;
	struct ais_ife_rdi_init_args      *rdi_cfg;
	struct cam_vfe_bus_ver3_hw_info   *bus_hw_info = NULL;
	struct cam_vfe_bus_ver3_reg_offset_bus_client  *client_regs = NULL;
	uint32_t client_idx, frame_inc;
	uint32_t width;
	int rc = 0;

	if (!hw_priv || !reserve_args || (arg_size !=
				sizeof(struct ais_ife_rdi_init_args))) {
		CAM_ERR(CAM_ISP, "Invalid input arguments");
		return -EINVAL;
	}

	rdi_cfg = (struct ais_ife_rdi_init_args *)reserve_args;
	if (rdi_cfg->path >= AIS_IFE_PATH_MAX) {
		CAM_ERR(CAM_ISP, "Invalid output path %d", rdi_cfg->path);
		return -EINVAL;
	}

	core_info = (struct ais_vfe_hw_core_info *)vfe_hw->core_info;
	rdi_path = &core_info->rdi_out[rdi_cfg->path];
	bus_hw_info = core_info->vfe_hw_info->bus_hw_info;
	client_idx = bus_hw_info->vfe_out_hw_info[rdi_cfg->path].wm_idx[0];
	client_regs = &bus_hw_info->bus_client_reg[client_idx];

	CAM_DBG(CAM_ISP, "Config RDI%d w:h:s = %u:%u:%u", rdi_cfg->path, rdi_cfg->out_cfg.width, rdi_cfg->out_cfg.height, rdi_cfg->out_cfg.stride);

	mutex_lock(&vfe_hw->hw_mutex);

	if (rdi_path->state >= AIS_ISP_RESOURCE_STATE_INIT_HW) {
		CAM_ERR(CAM_ISP, "RDI%d invalid state %d", rdi_cfg->path,
				rdi_path->state);
		rc = -EINVAL;
		goto EXIT;
	}

	rdi_path->secure_mode = rdi_cfg->out_cfg.secure_mode;

	/*disable pack as it is done in CSID*/
	cam_io_w(0x0, core_info->mem_base + client_regs->packer_cfg);

	width = ALIGNUP(rdi_cfg->out_cfg.width, 16) / 16;
	cam_io_w((rdi_cfg->out_cfg.height << 16) | width,
			core_info->mem_base + client_regs->image_cfg_0);
	cam_io_w(0, core_info->mem_base + client_regs->image_cfg_1);
	cam_io_w(rdi_cfg->out_cfg.stride,
			core_info->mem_base + client_regs->image_cfg_2);

	frame_inc = rdi_cfg->out_cfg.stride * rdi_cfg->out_cfg.height;
	frame_inc = CAM_36BIT_INTF_GET_IOVA_BASE(frame_inc);
	cam_io_w(frame_inc, core_info->mem_base + client_regs->frame_incr);

	cam_io_r_mb(
			core_info->mem_base + client_regs->irq_subsample_period);
	cam_io_r_mb(
			core_info->mem_base + client_regs->irq_subsample_pattern);
	cam_io_w_mb(0,
			core_info->mem_base + client_regs->irq_subsample_period);
	cam_io_w_mb(1,
			core_info->mem_base + client_regs->irq_subsample_pattern);
	cam_io_r_mb(
			core_info->mem_base + client_regs->framedrop_period);
	cam_io_r_mb(
			core_info->mem_base + client_regs->framedrop_pattern);
	cam_io_w_mb(rdi_cfg->out_cfg.frame_drop_period,
			core_info->mem_base + client_regs->framedrop_period);
	cam_io_w_mb(rdi_cfg->out_cfg.frame_drop_pattern,
			core_info->mem_base + client_regs->framedrop_pattern);

	rdi_path->batchConfig = rdi_cfg->out_cfg.batch_config;
	if (rdi_path->batchConfig.numBatchFrames < 1 || rdi_path->batchConfig.numBatchFrames > 4) {
		CAM_ERR(CAM_ISP, "invalid numBatchFrames %d", rdi_path->batchConfig.numBatchFrames);
		rdi_path->batchConfig.numBatchFrames = 1; //set numBatchFrames as default
	}

	rdi_path->state = AIS_ISP_RESOURCE_STATE_INIT_HW;
EXIT:
	mutex_unlock(&vfe_hw->hw_mutex);

	return rc;
}

int ais_vfe_start(void *hw_priv, void *start_args, uint32_t arg_size)
{
	struct ais_vfe_hw_core_info       *core_info = NULL;
	struct cam_hw_info                *vfe_hw  = hw_priv;
	struct ais_ife_rdi_start_args     *start_cmd;
	struct ais_vfe_rdi_output         *rdi_path;
	struct cam_vfe_top_ver4_hw_info   *top_hw_info = NULL;
	struct cam_vfe_bus_ver3_hw_info   *bus_hw_info = NULL;
	struct cam_irq_register_set       *bus_hw_irq_regs = NULL;
	struct cam_vfe_bus_ver3_reg_offset_bus_client  *client_regs = NULL;
	uint32_t irq_mask;
	uint32_t client_idx;
	int rc = 0;
	const uint32_t enable_debug_status_1 = 11 << 8;

	if (!hw_priv || !start_args ||
			(arg_size != sizeof(struct ais_ife_rdi_start_args))) {
		CAM_ERR(CAM_ISP, "Invalid input arguments");
		return -EINVAL;
	}

	start_cmd = (struct ais_ife_rdi_start_args *)start_args;
	if (start_cmd->path >= AIS_IFE_PATH_MAX) {
		CAM_ERR(CAM_ISP, "Invalid output path %d", start_cmd->path);
		return -EINVAL;
	}
	CAM_DBG(CAM_ISP, "start path %d", start_cmd->path);

	core_info = (struct ais_vfe_hw_core_info *)vfe_hw->core_info;
	rdi_path = &core_info->rdi_out[start_cmd->path];
	top_hw_info = core_info->vfe_hw_info->top_hw_info;
	bus_hw_info = core_info->vfe_hw_info->bus_hw_info;
	bus_hw_irq_regs = bus_hw_info->common_reg.irq_reg_info.irq_reg_set;
	client_idx = bus_hw_info->vfe_out_hw_info[start_cmd->path].wm_idx[0];
	client_regs = &bus_hw_info->bus_client_reg[client_idx];

	mutex_lock(&vfe_hw->hw_mutex);

	if (rdi_path->state != AIS_ISP_RESOURCE_STATE_INIT_HW) {
		CAM_ERR(CAM_ISP, "RDI%d invalid state %d", start_cmd->path,
				rdi_path->state);
		rc = -EINVAL;
		goto EXIT;
	}

	/* enable vfe top irqs */
	cam_io_r_mb(core_info->mem_base + core_info->vfe_hw_info->irq_hw_info->top_irq_reg->irq_reg_set[0].mask_reg_offset);
	cam_io_r_mb(core_info->mem_base + core_info->vfe_hw_info->irq_hw_info->top_irq_reg->irq_reg_set[1].mask_reg_offset);
	irq_mask = top_hw_info->rdi_hw_info[start_cmd->path].reg_data->sof_irq_mask |
		top_hw_info->rdi_hw_info[start_cmd->path].reg_data->epoch0_irq_mask |
		top_hw_info->rdi_hw_info[start_cmd->path].reg_data->eof_irq_mask;
	cam_io_w_mb(irq_mask, core_info->mem_base + core_info->vfe_hw_info->irq_hw_info->top_irq_reg->irq_reg_set[1].mask_reg_offset);
	irq_mask = top_hw_info->rdi_hw_info[start_cmd->path].reg_data->error_irq_mask | (1 << bus_hw_info->top_irq_shift);
	cam_io_w_mb(irq_mask, core_info->mem_base + core_info->vfe_hw_info->irq_hw_info->top_irq_reg->irq_reg_set[0].mask_reg_offset);

	/* enable vfe bus irqs */
	irq_mask = 0xD0000000;
	cam_io_w_mb(irq_mask, core_info->mem_base + bus_hw_irq_regs[0].mask_reg_offset);

	/* Enable WM */
	cam_io_w_mb(0x1, core_info->mem_base + client_regs->cfg);

	/* Enable constraint error detection */
	cam_io_w_mb(enable_debug_status_1,
			core_info->mem_base +
			client_regs->debug_status_cfg);

	rdi_path->state = AIS_ISP_RESOURCE_STATE_STREAMING;
EXIT:
	mutex_unlock(&vfe_hw->hw_mutex);

	return rc;
}

int ais_vfe_stop(void *hw_priv, void *stop_args, uint32_t arg_size)
{
	struct ais_vfe_hw_core_info       *core_info = NULL;
	struct cam_hw_info                *vfe_hw  = hw_priv;
	struct ais_ife_rdi_stop_args      *stop_cmd;
	struct ais_vfe_rdi_output         *rdi_path;
	struct cam_vfe_top_ver4_hw_info   *top_hw_info = NULL;
	struct cam_vfe_bus_ver3_hw_info   *bus_hw_info = NULL;
	struct cam_irq_register_set       *bus_hw_irq_regs = NULL;
	struct cam_vfe_bus_ver3_reg_offset_bus_client  *client_regs = NULL;
	uint32_t client_idx;
	int rc = 0;
	CAM_DBG(CAM_ISP, "");

	if (!hw_priv || !stop_args ||
			(arg_size != sizeof(struct ais_ife_rdi_stop_args))) {
		CAM_ERR(CAM_ISP, "Invalid input arguments");
		return -EINVAL;
	}

	core_info = (struct ais_vfe_hw_core_info *)vfe_hw->core_info;
	stop_cmd = (struct ais_ife_rdi_stop_args  *)stop_args;

	if (stop_cmd->path >= AIS_IFE_PATH_MAX) {
		CAM_ERR(CAM_ISP, "Invalid output path %d", stop_cmd->path);
		return -EINVAL;
	}

	core_info = (struct ais_vfe_hw_core_info *)vfe_hw->core_info;
	rdi_path = &core_info->rdi_out[stop_cmd->path];
	top_hw_info = core_info->vfe_hw_info->top_hw_info;
	bus_hw_info = core_info->vfe_hw_info->bus_hw_info;
	bus_hw_irq_regs = bus_hw_info->common_reg.irq_reg_info.irq_reg_set;
	client_idx = bus_hw_info->vfe_out_hw_info[stop_cmd->path].wm_idx[0];
	client_regs = &bus_hw_info->bus_client_reg[client_idx];

	mutex_lock(&vfe_hw->hw_mutex);

	if (rdi_path->state != AIS_ISP_RESOURCE_STATE_STREAMING &&
			rdi_path->state != AIS_ISP_RESOURCE_STATE_ERROR) {
		CAM_ERR(CAM_ISP, "RDI%d invalid state %d", stop_cmd->path,
				rdi_path->state);
		rc = -EINVAL;
		goto EXIT;
	}

	spin_lock(&rdi_path->buffer_lock);
	ais_clear_rdi_path(rdi_path);
	spin_unlock(&rdi_path->buffer_lock);

	/* Disable WM and reg-update */
	cam_io_w_mb(0x0, core_info->mem_base + client_regs->cfg);

	rdi_path->state = AIS_ISP_RESOURCE_STATE_INIT_HW;

EXIT:
	mutex_unlock(&vfe_hw->hw_mutex);

	return rc;
}

static uint8_t ais_vfe_get_num_missed_sof(
	uint64_t cur_sof,
	uint64_t prev_sof,
	uint64_t last_sof,
	uint64_t ts_delta)
{
	uint8_t miss_sof = 0;

	if (prev_sof == last_sof) {
		miss_sof = 0;
	} else if (prev_sof < last_sof) {
		//rollover case
		miss_sof = (int)(((U64_MAX - last_sof) + prev_sof + 1 +
					ts_delta/2) / ts_delta);
	} else {
		miss_sof = (int)((prev_sof - last_sof + ts_delta/2) / ts_delta);
	}

	return miss_sof;
}

static int ais_vfe_q_sof(struct ais_vfe_hw_core_info *core_info,
	enum ais_ife_output_path_id path,
	struct ais_sof_info_t *p_sof)
{
	struct ais_vfe_rdi_output *p_rdi = &core_info->rdi_out[path];
	struct ais_sof_info_t *p_sof_info = NULL;
	int rc = 0;

	if (!list_empty(&p_rdi->free_sof_info_list)) {
		if (p_rdi->last_sof_info.cur_sof_hw_ts == p_sof->cur_sof_hw_ts) {
			CAM_WARN(CAM_ISP, "I%d|R%d|F%llu: sof is expired.",
					core_info->vfe_idx, path, p_sof->frame_cnt);
			goto end;
		}

		spin_lock_bh(&p_rdi->buffer_lock);
		p_sof_info = list_first_entry(&p_rdi->free_sof_info_list,
				struct ais_sof_info_t, list);
		list_del_init(&p_sof_info->list);
		p_sof_info->frame_cnt = p_sof->frame_cnt;
		p_sof_info->sof_ts = p_sof->sof_ts;
		p_sof_info->cur_sof_hw_ts = p_sof->cur_sof_hw_ts;
		p_sof_info->prev_sof_hw_ts = p_sof->prev_sof_hw_ts;
		list_add_tail(&p_sof_info->list, &p_rdi->sof_info_q);
		p_rdi->num_sof_info_q++;
		spin_unlock_bh(&p_rdi->buffer_lock);

		trace_ais_isp_vfe_q_sof(core_info->vfe_idx, path,
				p_sof->frame_cnt, p_sof->cur_sof_hw_ts);

		CAM_DBG(CAM_ISP, "I%d|R%d|F%llu: sof %llu",
				core_info->vfe_idx, path, p_sof->frame_cnt,
				p_sof_info->cur_sof_hw_ts);
	} else {
		rc = -1;

		CAM_DBG(CAM_ISP,
				"I%d|R%d|F%llu: free timestamp empty (%d) sof %llu",
				core_info->vfe_idx, path, p_sof->frame_cnt,
				p_rdi->num_buffer_hw_q, p_sof->cur_sof_hw_ts);
	}

end:
	return rc;
}

static void ais_vfe_handle_sof_rdi(struct ais_vfe_hw_core_info *core_info,
		struct ais_vfe_hw_work_data *work_data,
		enum ais_ife_output_path_id path)
{
	struct ais_vfe_rdi_output *p_rdi = &core_info->rdi_out[path];
	uint64_t cur_sof_hw_ts = work_data->ts_hw[path].cur_sof_ts;
	uint64_t prev_sof_hw_ts = work_data->ts_hw[path].prev_sof_ts;

	p_rdi->frame_cnt++;
	core_info->event.msg.reserved = sizeof(struct ais_ife_event_data);
	if (p_rdi->num_buffer_hw_q) {
		struct ais_sof_info_t sof = {};
		uint64_t ts_delta;
		uint8_t miss_sof = 0;

		if (cur_sof_hw_ts < prev_sof_hw_ts)
			ts_delta = cur_sof_hw_ts +
				(U64_MAX - prev_sof_hw_ts);
		else
			ts_delta = cur_sof_hw_ts - prev_sof_hw_ts;


		//check any missing SOFs
		if (p_rdi->frame_cnt > 1) {
			if (ts_delta == 0) {
				CAM_ERR(CAM_ISP, "IFE%d RDI%d ts_delta is 0",
						core_info->vfe_idx, path);
			} else {
				miss_sof = ais_vfe_get_num_missed_sof(
						cur_sof_hw_ts,
						prev_sof_hw_ts,
						p_rdi->last_sof_info.cur_sof_hw_ts,
						ts_delta);

				CAM_DBG(CAM_ISP,
						"I%d R%d miss_sof %u prev %llu last %llu cur %llu",
						core_info->vfe_idx, path,
						miss_sof, prev_sof_hw_ts,
						p_rdi->last_sof_info.cur_sof_hw_ts,
						cur_sof_hw_ts);
			}
		}

		trace_ais_isp_vfe_sof(core_info->vfe_idx, path,
				&work_data->ts_hw[path],
				p_rdi->num_buffer_hw_q, miss_sof);

		/*if (p_rdi->frame_cnt == 1 && prev_sof_hw_ts != 0) {
				//enq missed first frame
				sof.sof_ts = work_data->ts;
				sof.cur_sof_hw_ts = prev_sof_hw_ts;
				sof.frame_cnt = p_rdi->frame_cnt++;

				ais_vfe_q_sof(core_info, path, &sof);
			} else if (miss_sof > 0) {
				if (miss_sof > 1) {
					int i = 0;
					int miss_idx = miss_sof - 1;

					for (i = 0; i < (miss_sof - 1); i++) {
						sof.sof_ts = work_data->ts;
						sof.cur_sof_hw_ts = prev_sof_hw_ts -
							(ts_delta * miss_idx);
						sof.frame_cnt = p_rdi->frame_cnt++;

						ais_vfe_q_sof(core_info, path, &sof);

						miss_idx--;
					}
				}

				//enq prev
				sof.sof_ts = work_data->ts;
				sof.cur_sof_hw_ts = prev_sof_hw_ts;
				sof.frame_cnt = p_rdi->frame_cnt++;

				ais_vfe_q_sof(core_info, path, &sof);
			}*/

		//enq curr
		sof.sof_ts = work_data->ts;
		sof.cur_sof_hw_ts = cur_sof_hw_ts;
		sof.frame_cnt = p_rdi->frame_cnt;

		ais_vfe_q_sof(core_info, path, &sof);

	} else {
		trace_ais_isp_vfe_sof(core_info->vfe_idx, path,
				&work_data->ts_hw[path],
				p_rdi->num_buffer_hw_q, 0);

		CAM_DBG(CAM_ISP, "I%d R%d Flush SOF (%d) HW Q empty",
				core_info->vfe_idx, path,
				p_rdi->num_sof_info_q);

		if (p_rdi->num_sof_info_q) {
			struct ais_sof_info_t *p_sof_info;

			spin_lock_bh(&p_rdi->buffer_lock);
			while (!list_empty(&p_rdi->sof_info_q)) {
				p_sof_info = list_first_entry(
						&p_rdi->sof_info_q,
						struct ais_sof_info_t, list);
				list_del_init(&p_sof_info->list);
				list_add_tail(&p_sof_info->list,
						&p_rdi->free_sof_info_list);
			}
			p_rdi->num_sof_info_q = 0;
			spin_unlock_bh(&p_rdi->buffer_lock);
		}

		trace_ais_isp_vfe_error(core_info->vfe_idx,
				path, 1, 0);

		//send warning
		core_info->event.msg.type = AIS_IFE_MSG_OUTPUT_WARNING;
		core_info->event.msg.path = path;
		core_info->event.u.err_msg.reserved = 0;

		core_info->event_cb(core_info->event_cb_priv,
				&core_info->event);

	}

	p_rdi->last_sof_info.cur_sof_hw_ts = cur_sof_hw_ts;

	//send sof only for current frame
	core_info->event.msg.type = AIS_IFE_MSG_SOF;
	core_info->event.msg.path = path;
	core_info->event.msg.frame_id = p_rdi->frame_cnt;
	core_info->event.u.sof_msg.hw_ts = cur_sof_hw_ts;

	core_info->event_cb(core_info->event_cb_priv,
			&core_info->event);
}

static int ais_vfe_handle_sof(
	struct ais_vfe_hw_core_info *core_info,
	struct ais_vfe_hw_work_data *work_data)
{
	struct ais_vfe_rdi_output *p_rdi;
	struct cam_vfe_top_ver4_hw_info   *top_hw_info = NULL;
	struct cam_vfe_ver4_path_reg_data *rdi_reg;
	int path =  0;
	int rc = 0;

	CAM_DBG(CAM_ISP, "IFE%d SOF RDIs 0x%x", core_info->vfe_idx,
			work_data->path);

	top_hw_info = core_info->vfe_hw_info->top_hw_info;
	for (path = 0; path < top_hw_info->num_rdi; path++) {
		rdi_reg = top_hw_info->rdi_hw_info[path].reg_data;

		if (!(work_data->path & rdi_reg->sof_irq_mask))
			continue;

		p_rdi = &core_info->rdi_out[path];
		if (p_rdi->state != AIS_ISP_RESOURCE_STATE_STREAMING)
			continue;

		AIS_ATRACE_BEGIN("SOF_%u_%u_%lu",
				core_info->vfe_idx, path, p_rdi->frame_cnt);
		ais_vfe_handle_sof_rdi(core_info, work_data, path);
		AIS_ATRACE_END("SOF_%u_%u_%lu",
				core_info->vfe_idx, path, p_rdi->frame_cnt);

		//enq buffers
		spin_lock_bh(&p_rdi->buffer_lock);
		ais_vfe_q_bufs_to_hw(core_info, path);
		spin_unlock_bh(&p_rdi->buffer_lock);
	}

	return rc;
}

static void ais_vfe_bus_handle_client_frame_done(
	struct ais_vfe_hw_core_info *core_info,
	enum ais_ife_output_path_id path,
	uint32_t last_addr)
{
	struct ais_vfe_rdi_output         *rdi_path = NULL;
	struct ais_vfe_buffer_t           *vfe_buf = NULL, *buf_tmp;
	struct cam_vfe_bus_ver3_hw_info   *bus_hw_info = NULL;
	uint64_t                           frame_cnt = 0;
	uint64_t                           sof_ts = 0;
	uint64_t                           cur_sof_hw_ts;
	uint32_t iova_addr;
	bool last_addr_match = false;
	uint32_t i = 0;

	CAM_DBG(CAM_ISP, "I%d|R%d last_addr 0x%x",
			core_info->vfe_idx, path, last_addr);

	if (last_addr == 0) {
		CAM_ERR(CAM_ISP, "I%d|R%d null last_addr",
				core_info->vfe_idx, path);
		return;
	}

	rdi_path = &core_info->rdi_out[path];
	bus_hw_info = core_info->vfe_hw_info->bus_hw_info;

	core_info->event.msg.type = AIS_IFE_MSG_FRAME_DONE;
	core_info->event.msg.path = path;
	core_info->event.msg.reserved = sizeof(struct ais_ife_event_data);

	while (rdi_path->num_buffer_hw_q && !last_addr_match) {
		struct ais_sof_info_t *p_sof_info = NULL;
		bool is_sof_match = false;

		spin_lock_bh(&rdi_path->buffer_lock);
		if (list_empty(&rdi_path->buffer_hw_q)) {
			CAM_DBG(CAM_ISP, "I%d|R%d: FD while HW Q empty",
					core_info->vfe_idx, path);
			spin_unlock_bh(&rdi_path->buffer_lock);
			break;
		}

		list_for_each_entry_safe(vfe_buf, buf_tmp, &rdi_path->buffer_hw_q, list) {
			if (cam_smmu_is_expanded_memory()) {
				iova_addr = CAM_36BIT_INTF_GET_IOVA_BASE(vfe_buf->iova_addr);
			} else {
				iova_addr = vfe_buf->iova_addr;
			}
			if (last_addr == iova_addr) {
				last_addr_match = true;
				list_del_init(&vfe_buf->list);
				--rdi_path->num_buffer_hw_q;
				break;
			}
			else
				CAM_ERR(CAM_ISP, "IFE%d buf %d did not match addr 0x%x",
						core_info->vfe_idx, vfe_buf->bufIdx, iova_addr);
		}

		if (!last_addr_match) {
			spin_unlock_bh(&rdi_path->buffer_lock);
			break;
		}

		CAM_DBG(CAM_ISP, "I%d|R%d BUF DQ %d (0x%x) FIFO:%d|0x%x",
				core_info->vfe_idx, path,
				vfe_buf->bufIdx, vfe_buf->iova_addr,
				rdi_path->num_buffer_hw_q, last_addr);

		if (!list_empty(&rdi_path->sof_info_q)) {
			while (!is_sof_match &&
					!list_empty(&rdi_path->sof_info_q)) {
				p_sof_info =
					list_first_entry(&rdi_path->sof_info_q,
							struct ais_sof_info_t, list);
				list_del_init(&p_sof_info->list);
				rdi_path->num_sof_info_q--;
				if (p_sof_info->cur_sof_hw_ts >
						vfe_buf->ts_hw.cur_sof_ts) {
					is_sof_match = true;
					break;
				}
				list_add_tail(&p_sof_info->list,
						&rdi_path->free_sof_info_list);
			}

			if (!is_sof_match) {
				p_sof_info = NULL;
				CAM_ERR(CAM_ISP,
						"I%d|R%d: can't find the match sof",
						core_info->vfe_idx, path);
			}

		} else
			CAM_ERR(CAM_ISP, "I%d|R%d: SOF info Q is empty",
					core_info->vfe_idx, path);

		if (p_sof_info) {
			frame_cnt = p_sof_info->frame_cnt;
			sof_ts = p_sof_info->sof_ts;
			cur_sof_hw_ts = p_sof_info->cur_sof_hw_ts;
			list_add_tail(&p_sof_info->list,
					&rdi_path->free_sof_info_list);
		} else {
			struct ais_ife_rdi_get_timestamp_args get_ts;
			struct ais_ife_rdi_timestamps ts_hw;
			struct timespec64 ts;

			get_ts.path = path;
			get_ts.ts = &ts_hw;
			core_info->csid_hw->hw_ops.process_cmd(
					core_info->csid_hw->hw_priv,
					AIS_IFE_CSID_CMD_GET_TIME_STAMP,
					&get_ts,
					sizeof(get_ts));

			CAM_DBG(CAM_ISP, "No matched SOF info, get current [%llu, %llu]",
					ts_hw.prev_sof_ts, ts_hw.cur_sof_ts);
			if (rdi_path->last_sof_info.cur_sof_hw_ts < ts_hw.prev_sof_ts) {
				ktime_get_boottime_ts64(&ts);
				frame_cnt = rdi_path->frame_cnt + 1;
				sof_ts = (uint64_t)((ts.tv_sec * 1000000000) + ts.tv_nsec);
				cur_sof_hw_ts = ts_hw.prev_sof_ts;
				rdi_path->last_sof_info.cur_sof_hw_ts = ts_hw.prev_sof_ts;
			} else if (rdi_path->last_sof_info.cur_sof_hw_ts < ts_hw.cur_sof_ts) {
				ktime_get_boottime_ts64(&ts);
				frame_cnt = rdi_path->frame_cnt + 1;
				sof_ts = (uint64_t)((ts.tv_sec * 1000000000) + ts.tv_nsec);
				cur_sof_hw_ts = ts_hw.cur_sof_ts;
				rdi_path->last_sof_info.cur_sof_hw_ts = ts_hw.cur_sof_ts;
			} else {
				frame_cnt = sof_ts = cur_sof_hw_ts = 0;
			}
		}

		list_add_tail(&vfe_buf->list, &rdi_path->free_buffer_list);
		spin_unlock_bh(&rdi_path->buffer_lock);

		trace_ais_isp_vfe_buf_done(core_info->vfe_idx, path,
				vfe_buf->bufIdx,
				frame_cnt,
				rdi_path->num_buffer_hw_q,
				last_addr_match);

		rdi_path->batchFrameInfo[vfe_buf->batchId].batchId = vfe_buf->batchId;
		rdi_path->batchFrameInfo[vfe_buf->batchId].frameId = frame_cnt;
		rdi_path->batchFrameInfo[vfe_buf->batchId].hwTimestamp = cur_sof_hw_ts;

		if (vfe_buf->batchId == (rdi_path->batchConfig.numBatchFrames - 1)) {
			core_info->event.u.frame_msg.buf_idx = vfe_buf->bufIdx;
			core_info->event.u.frame_msg.num_batch_frames =
				rdi_path->batchConfig.numBatchFrames;
			core_info->event.u.frame_msg.ts = sof_ts;
			core_info->event.msg.frame_id =
				rdi_path->batchFrameInfo[i].frameId;
			for (i = 0; i < rdi_path->batchConfig.numBatchFrames; i++) {
				core_info->event.u.frame_msg.hw_ts[i] =
					rdi_path->batchFrameInfo[i].hwTimestamp;
			}
			core_info->event_cb(core_info->event_cb_priv,
					&core_info->event);

			CAM_DBG(CAM_ISP, "I%d|R%d|F%u: si [%llu, %llu]",
					core_info->vfe_idx, path,
					core_info->event.msg.frame_id,
					sof_ts,
					core_info->event.u.frame_msg.hw_ts[0]);
		}
	}

	if (!last_addr_match) {
		CAM_ERR(CAM_ISP, "IFE%d BUF| RDI%d NO MATCH addr 0x%x",
				core_info->vfe_idx, path, last_addr);

		trace_ais_isp_vfe_error(core_info->vfe_idx, path, 1, 1);

		//send warning
		core_info->event.msg.type = AIS_IFE_MSG_OUTPUT_WARNING;
		core_info->event.msg.path = path;
		core_info->event.u.err_msg.reserved = 1;

		core_info->event_cb(core_info->event_cb_priv,
				&core_info->event);
	}

	/* Flush SOF info Q if HW Buffer Q is empty */
	if (rdi_path->num_buffer_hw_q == 0) {
		struct ais_sof_info_t *p_sof_info = NULL;

		CAM_DBG(CAM_ISP, "I%d|R%d|F%llu: Flush SOF (%d) HW Q empty",
				core_info->vfe_idx, path, frame_cnt,
				rdi_path->num_sof_info_q);

		spin_lock_bh(&rdi_path->buffer_lock);
		while (!list_empty(&rdi_path->sof_info_q)) {
			p_sof_info = list_first_entry(&rdi_path->sof_info_q,
					struct ais_sof_info_t, list);
			list_del_init(&p_sof_info->list);
			list_add_tail(&p_sof_info->list,
					&rdi_path->free_sof_info_list);
		}

		rdi_path->num_sof_info_q = 0;
		spin_unlock_bh(&rdi_path->buffer_lock);

		trace_ais_isp_vfe_error(core_info->vfe_idx, path, 1, 0);

		//send warning
		core_info->event.msg.type = AIS_IFE_MSG_OUTPUT_WARNING;
		core_info->event.msg.path = path;
		core_info->event.u.err_msg.reserved = 0;

		core_info->event_cb(core_info->event_cb_priv,
				&core_info->event);
	}

	spin_lock_bh(&rdi_path->buffer_lock);

	ais_vfe_q_bufs_to_hw(core_info, path);

	spin_unlock_bh(&rdi_path->buffer_lock);
}

static int ais_vfe_bus_handle_frame_done(
	struct ais_vfe_hw_core_info *core_info,
	struct ais_vfe_hw_work_data *work_data)
{
	struct cam_vfe_bus_ver3_hw_info   *bus_hw_info = NULL;
	struct cam_vfe_bus_ver3_reg_offset_bus_client  *client_regs = NULL;
	struct ais_vfe_rdi_output *p_rdi = &core_info->rdi_out[0];
	uint32_t client_mask = work_data->bus_wr_status[1];
	uint32_t i, client, comp_grp_shift;
	int rc = 0;

	CAM_DBG(CAM_ISP, "VFE%d Frame Done clients 0x%x",
			core_info->vfe_idx, client_mask);

	bus_hw_info = core_info->vfe_hw_info->bus_hw_info;
	for (i = 0 ; i < AIS_IFE_PATH_MAX; i++) {
		p_rdi = &core_info->rdi_out[i];

		if (p_rdi->state != AIS_ISP_RESOURCE_STATE_STREAMING)
			continue;
		client = bus_hw_info->vfe_out_hw_info[i].wm_idx[0];
		client_regs =
			&bus_hw_info->bus_client_reg[client];
		comp_grp_shift = (client_regs->comp_group +
				bus_hw_info->vfe_out_hw_info[i].bufdone_shift +
				bus_hw_info->comp_done_shift);
		if (client_mask & (0x1 << comp_grp_shift)) {
			//process frame done
			AIS_ATRACE_BEGIN("FD_%u_%u_%lu",
					core_info->vfe_idx, i, p_rdi->frame_cnt);
			ais_vfe_bus_handle_client_frame_done(core_info,
					i, work_data->last_addr[i]);
			AIS_ATRACE_END("FD_%u_%u_%lu",
					core_info->vfe_idx, i, p_rdi->frame_cnt);
		}
	}

	return rc;
}

static int ais_vfe_handle_error(
	struct ais_vfe_hw_core_info *core_info,
	struct ais_vfe_hw_work_data *work_data)
{
	struct ais_vfe_rdi_output *p_rdi;
	int path =  0;
	int rc = 0;

	CAM_ERR(CAM_ISP, "IFE%d ERROR on RDIs 0x%x", core_info->vfe_idx,
                    work_data->path);

	trace_ais_isp_vfe_error(core_info->vfe_idx,
            work_data->path, 0, 0);

	for (path = 0; path < AIS_IFE_PATH_MAX; path++) {
		if (!(work_data->path & (1 << path)))
			continue;

		p_rdi = &core_info->rdi_out[path];

		if (p_rdi->state != AIS_ISP_RESOURCE_STATE_STREAMING)
			continue;

		p_rdi->state = AIS_ISP_RESOURCE_STATE_ERROR;

		core_info->event.msg.type = AIS_IFE_MSG_OUTPUT_ERROR;
		core_info->event.msg.path = path;
		core_info->event.msg.reserved = sizeof(struct ais_ife_event_data);

		core_info->event_cb(core_info->event_cb_priv,
				&core_info->event);
	}
	return rc;
}

static int ais_vfe_process_irq_bh(void *priv, void *data)
{
	struct ais_vfe_hw_work_data   *work_data;
	struct cam_hw_info            *vfe_hw;
	struct ais_vfe_hw_core_info   *core_info;
	int rc = 0;

	vfe_hw = (struct cam_hw_info *)priv;
	if (!vfe_hw) {
		CAM_ERR(CAM_ISP, "Invalid parameters");
		return -EINVAL;
	}

	core_info = (struct ais_vfe_hw_core_info *)vfe_hw->core_info;
	if (!core_info->event_cb) {
		CAM_ERR(CAM_ISP, "hw_idx %d Error Cb not registered",
				core_info->vfe_idx);
		return -EINVAL;
	}

	work_data = (struct ais_vfe_hw_work_data *)data;

	trace_ais_isp_irq_process(core_info->vfe_idx, work_data->evt_type, 1);
	CAM_DBG(CAM_ISP, "VFE[%d] event %d",
			core_info->vfe_idx, work_data->evt_type);

	core_info->event.msg.idx = core_info->vfe_idx;
	core_info->event.msg.boot_ts = work_data->ts;

	switch (work_data->evt_type) {
		case AIS_VFE_HW_IRQ_EVENT_SOF:
			AIS_ATRACE_BEGIN("SOF_%d", core_info->vfe_idx);
			rc = ais_vfe_handle_sof(core_info, work_data);
			AIS_ATRACE_END("SOF_%d", core_info->vfe_idx);
			break;
		case AIS_VFE_HW_IRQ_EVENT_BUS_WR:
			//rc = ais_vfe_handle_bus_wr_irq(vfe_hw, core_info, work_data);
			break;
		case AIS_VFE_HW_IRQ_EVENT_ERROR:
			rc = ais_vfe_handle_error(core_info, work_data);
			break;
		case AIS_VFE_HW_IRQ_EVENT_BUF_DONE:
			AIS_ATRACE_BEGIN("FD_%d", core_info->vfe_idx);
			ais_vfe_bus_handle_frame_done(core_info, work_data);
			AIS_ATRACE_END("FD_%d", core_info->vfe_idx);
			break;
		default:
			CAM_ERR(CAM_ISP, "VFE[%d] invalid event type %d",
					core_info->vfe_idx, work_data->evt_type);
			break;
	}

	trace_ais_isp_irq_process(core_info->vfe_idx, work_data->evt_type, 2);

	return rc;
}

static int ais_vfe_dispatch_irq(struct cam_hw_info *vfe_hw,
		struct ais_vfe_hw_work_data *p_work)
{
	struct ais_vfe_hw_core_info *core_info;
	struct ais_vfe_hw_work_data *work_data;
	struct crm_workq_task *task;
	int rc = 0;

	core_info = (struct ais_vfe_hw_core_info *)vfe_hw->core_info;

	CAM_DBG(CAM_ISP, "VFE[%d] event %d",
		core_info->vfe_idx, p_work->evt_type);

	task = cam_req_mgr_workq_get_task(core_info->workq);
	if (!task) {
		CAM_ERR_RATE_LIMIT(CAM_ISP,
			"I%d Can not get task for worker, cancel SOF evt",
			core_info->vfe_idx);

		/*cam_req_mgr_workq_cancel_task(core_info->workq,
						ais_vfe_irq_cancel_task_filter);*/

		if (p_work->evt_type == AIS_VFE_HW_IRQ_EVENT_SOF) {
			CAM_DBG(CAM_ISP, "I%d So discard this SOF event",
					core_info->vfe_idx);
			return -ENOMEM;
		}

		task = cam_req_mgr_workq_get_task(core_info->workq);
		if (!task) {
			CAM_ERR_RATE_LIMIT(CAM_ISP,
					"I%d Still can not get task for worker",
					core_info->vfe_idx);
			return -ENOMEM;
		}
	}
	work_data = (struct ais_vfe_hw_work_data *)task->payload;
	*work_data = *p_work;

	trace_ais_isp_irq_process(core_info->vfe_idx, p_work->evt_type, 0);

	task->process_cb = ais_vfe_process_irq_bh;
	rc = cam_req_mgr_workq_enqueue_task(task, vfe_hw,
		CRM_TASK_PRIORITY_0);

	return rc;
}

int ais_vfe_process_buf_done(struct cam_hw_info *vfe_hw,
	struct ais_ife_proc_buf_done_args *buf_done)
{
	int rc = 0;
	struct ais_vfe_hw_work_data work_data;
	struct ais_vfe_hw_core_info   *core_info;
	struct cam_vfe_bus_ver3_hw_info   *bus_hw_info = NULL;
	struct cam_vfe_bus_ver3_reg_offset_bus_client  *client_regs = NULL;
	uint32_t i, client = 0, comp_grp_shift = 0;
	CAM_DBG(CAM_ISP, "got buf done irq 0x%08x.", buf_done->irq_status);

	core_info = (struct ais_vfe_hw_core_info *)vfe_hw->core_info;
	bus_hw_info = core_info->vfe_hw_info->bus_hw_info;

	work_data.evt_type = AIS_VFE_HW_IRQ_EVENT_BUF_DONE;
	work_data.bus_wr_status[1] = buf_done->irq_status;
	if (buf_done->irq_status) {
		for (i = 0 ; i < AIS_IFE_PATH_MAX; i++) {
			client = bus_hw_info->vfe_out_hw_info[i].wm_idx[0];
			client_regs =
				&bus_hw_info->bus_client_reg[client];
			comp_grp_shift = (client_regs->comp_group +
					bus_hw_info->vfe_out_hw_info[i].bufdone_shift +
					bus_hw_info->comp_done_shift);
			if (buf_done->irq_status & (0x1 << comp_grp_shift)) {
				CAM_DBG(CAM_ISP, "got buf done irq for path %u", i);
				work_data.last_addr[i] = cam_io_r(
						core_info->mem_base +
						client_regs->addr_status_0);
			}
		}
	}

	ais_vfe_dispatch_irq(vfe_hw, &work_data);

	return rc;
}

static void ais_req_mgr_process_workq_cam_ife_worker(struct work_struct *w)
{
	cam_req_mgr_process_workq(w);
}

irqreturn_t ais_vfe_irq(int irq_num, void *data)
{
	struct cam_hw_info            *vfe_hw;
	struct ais_vfe_hw_core_info   *core_info;
	struct cam_vfe_top_ver4_hw_info   *top_hw_info = NULL;
	struct cam_vfe_bus_ver3_hw_info   *bus_hw_info = NULL;
	struct ais_vfe_hw_work_data work_data;
	uint32_t ife_status[2] = {};
	uint32_t bus_hw_status;
	int path =  0;

	if (!data)
		return IRQ_NONE;

	vfe_hw = (struct cam_hw_info *)data;
	core_info = (struct ais_vfe_hw_core_info *)vfe_hw->core_info;
	top_hw_info = core_info->vfe_hw_info->top_hw_info;
	bus_hw_info = core_info->vfe_hw_info->bus_hw_info;
	CAM_DBG(CAM_ISP, "VFE irq handling");

	ife_status[0] = cam_io_r_mb(core_info->mem_base + core_info->vfe_hw_info->irq_hw_info->top_irq_reg->irq_reg_set[0].status_reg_offset);
	ife_status[1] = cam_io_r_mb(core_info->mem_base + core_info->vfe_hw_info->irq_hw_info->top_irq_reg->irq_reg_set[1].status_reg_offset);
	bus_hw_status = cam_io_r_mb(core_info->mem_base + bus_hw_info->common_reg.irq_reg_info.irq_reg_set[0].status_reg_offset);

	cam_io_w_mb(ife_status[0], core_info->mem_base + core_info->vfe_hw_info->irq_hw_info->top_irq_reg->irq_reg_set[0].clear_reg_offset);
	cam_io_w_mb(ife_status[1], core_info->mem_base + core_info->vfe_hw_info->irq_hw_info->top_irq_reg->irq_reg_set[1].clear_reg_offset);
	cam_io_w_mb(core_info->vfe_hw_info->irq_hw_info->top_irq_reg->global_clear_bitmask,
			core_info->mem_base + core_info->vfe_hw_info->irq_hw_info->top_irq_reg->global_irq_cmd_offset);

	cam_io_w_mb(bus_hw_status, core_info->mem_base + bus_hw_info->common_reg.irq_reg_info.irq_reg_set[0].clear_reg_offset);
	cam_io_w_mb(bus_hw_info->common_reg.irq_reg_info.global_clear_bitmask,
			core_info->mem_base + bus_hw_info->common_reg.irq_reg_info.global_irq_cmd_offset);

	trace_ais_isp_vfe_irq_activated(core_info->vfe_idx,
			ife_status[0], ife_status[1]);
	CAM_DBG(CAM_ISP, "VFE%d top status 0x%x 0x%x, bus status 0x%x", core_info->vfe_idx,
			ife_status[0], ife_status[1], bus_hw_status);

	if (ife_status[1]) {
		struct cam_vfe_ver4_path_reg_data *rdi_reg;
		struct ais_ife_rdi_get_timestamp_args get_ts;
		struct timespec64 ts;
		bool has_sof = false;

		ktime_get_boottime_ts64(&ts);
		work_data.ts =
			(uint64_t)((ts.tv_sec * 1000000000) + ts.tv_nsec);

		for (path = 0; path < top_hw_info->num_rdi; path++) {
			rdi_reg = top_hw_info->rdi_hw_info[path].reg_data;

			if (ife_status[1] & rdi_reg->sof_irq_mask) {
				has_sof = true;
				get_ts.path = path;
				get_ts.ts = &work_data.ts_hw[path];
				core_info->csid_hw->hw_ops.process_cmd(
						core_info->csid_hw->hw_priv,
						AIS_IFE_CSID_CMD_GET_TIME_STAMP,
						&get_ts,
						sizeof(get_ts));
			}
		}

		if (has_sof) {
			work_data.path = ife_status[1];
			work_data.evt_type = AIS_VFE_HW_IRQ_EVENT_SOF;
			ais_vfe_dispatch_irq(vfe_hw, &work_data);
		}
	}

	if (bus_hw_status) {
		uint32_t bus_overflow_status, bus_violation_status, image_size_violation_status;
		bus_overflow_status =
			cam_io_r_mb(core_info->mem_base +
			top_hw_info->common_reg->bus_overflow_status);
		bus_violation_status =
			cam_io_r_mb(core_info->mem_base +
			top_hw_info->common_reg->bus_violation_status);
		image_size_violation_status =
			cam_io_r_mb(core_info->mem_base +
			bus_hw_info->common_reg.image_size_violation_status);

		CAM_ERR(CAM_ISP, "bus hw error 0x%x", bus_hw_status);
		CAM_ERR(CAM_ISP, "check status: bus_overflow_status=0x%x "
			"bus_violation_status = 0x%x image_size_violation_status = 0x%x",
			bus_overflow_status, bus_violation_status, image_size_violation_status);

		work_data.path = 0xF;
		work_data.evt_type = AIS_VFE_HW_IRQ_EVENT_ERROR;
		ais_vfe_dispatch_irq(vfe_hw, &work_data);
	}

	return IRQ_HANDLED;
}

int ais_vfe_core_init(struct ais_vfe_hw_core_info  *core_info,
	struct cam_hw_soc_info                     *soc_info,
	struct cam_hw_intf                         *hw_intf,
	struct cam_vfe_hw_info                     *vfe_hw_info)
{
	int rc = 0;
	int i;
	char worker_name[128];

	CAM_DBG(CAM_ISP, "Enter");

	core_info->vfe_idx = soc_info->index;
	core_info->mem_base =
		CAM_SOC_GET_REG_MAP_START(soc_info, VFE_CORE_BASE_IDX);

	spin_lock_init(&core_info->spin_lock);

	for (i = 0; i < AIS_IFE_PATH_MAX; i++) {
		struct ais_vfe_rdi_output *p_rdi = &core_info->rdi_out[i];

		spin_lock_init(&p_rdi->buffer_lock);
		ais_clear_rdi_path(p_rdi);
		p_rdi->state = AIS_ISP_RESOURCE_STATE_AVAILABLE;
	}

	scnprintf(worker_name, sizeof(worker_name),
			"vfe%u_worker", core_info->vfe_idx);
	CAM_DBG(CAM_ISP, "Create VFE worker %s", worker_name);
	rc = cam_req_mgr_workq_create(worker_name,
			AIS_VFE_WORKQ_NUM_TASK,
			&core_info->workq, CRM_WORKQ_USAGE_IRQ, 0,
			ais_req_mgr_process_workq_cam_ife_worker);
	if (rc) {
		CAM_ERR(CAM_ISP, "Unable to create a workq, rc=%d", rc);
		goto EXIT;
	}

	core_info->work_data = kcalloc(AIS_VFE_WORKQ_NUM_TASK,
			sizeof(struct ais_vfe_hw_work_data), GFP_KERNEL);
	for (i = 0; i < AIS_VFE_WORKQ_NUM_TASK; i++)
		core_info->workq->task.pool[i].payload =
			&core_info->work_data[i];

	ais_vfe_create_debugfs_entry(core_info);

EXIT:
	return rc;
}

int ais_vfe_core_deinit(struct ais_vfe_hw_core_info  *core_info,
	struct cam_vfe_hw_info                       *vfe_hw_info)
{
	int                rc = 0;
	int                i;
	unsigned long      flags;

	cam_req_mgr_workq_destroy(&core_info->workq);

	spin_lock_irqsave(&core_info->spin_lock, flags);

	for (i = 0; i < AIS_IFE_PATH_MAX; i++) {
		struct ais_vfe_rdi_output *p_rdi = &core_info->rdi_out[i];

		ais_clear_rdi_path(p_rdi);
		p_rdi->state = AIS_ISP_RESOURCE_STATE_AVAILABLE;
	}

	spin_unlock_irqrestore(&core_info->spin_lock, flags);

	ais_vfe_remove_debugfs_entry(core_info);

	return rc;
}
