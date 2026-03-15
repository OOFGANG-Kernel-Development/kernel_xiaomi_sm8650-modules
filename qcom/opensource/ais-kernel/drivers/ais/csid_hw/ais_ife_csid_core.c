#include <linux/iopoll.h>
#include <linux/slab.h>
#include <linux/debugfs.h>
#include <linux/file.h>
#include <linux/fs.h>

#include <media/cam_isp.h>
#include <media/cam_defs.h>

#include "cam_req_mgr_workq.h"
#include "ais_ife_csid_core.h"
#include "ais_vfe_hw_intf.h"

/* Timeout values in usec */
#define AIS_IFE_CSID_TIMEOUT_SLEEP_US                  1000
#define AIS_IFE_CSID_TIMEOUT_ALL_US                    100000

#define AIS_IFE_CSID_RESET_TIMEOUT_MS                  100

/*
 * Constant Factors needed to change QTimer ticks to nanoseconds
 * QTimer Freq = 19.2 MHz
 * Time(us) = ticks/19.2
 * Time(ns) = ticks/19.2 * 1000
 */
#define AIS_IFE_CSID_QTIMER_MUL_FACTOR                 10000
#define AIS_IFE_CSID_QTIMER_DIV_FACTOR                 192

/* Max number of sof irq's triggered in case of SOF freeze */
#define AIS_CSID_IRQ_SOF_DEBUG_CNT_MAX 12

/* Max CSI Rx irq error count threshold value */
#define AIS_IFE_CSID_MAX_IRQ_ERROR_COUNT               5

static ssize_t ais_ife_csid_debug_reg_read(struct file *file,
	char __user *user_buf, size_t count, loff_t *ppos)
{
	char    *buf;
	ssize_t  rc;
	struct ais_ife_csid_hw *csid_hw =
		(struct ais_ife_csid_hw *)file->private_data;
	struct cam_hw_soc_info                *soc_info;

	if (!csid_hw)
		return -EINVAL;

	soc_info = &csid_hw->hw_info->soc_info;
	buf = kzalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	csid_hw->reg_val = cam_io_r_mb(soc_info->reg_map[0].mem_base + csid_hw->offset);
	rc = snprintf(buf, count, "0x%x\n", csid_hw->reg_val);

	rc = simple_read_from_buffer(user_buf, count, ppos, buf, rc);

	kfree(buf);
	return rc;
}

static ssize_t ais_ife_csid_debug_reg_write(struct file *file,
	const char __user *user_buf, size_t count, loff_t *ppos)
{
	char    buf[128];
	char   *start = buf;
	size_t  buf_size;
	struct ais_ife_csid_hw *csid_hw =
		(struct ais_ife_csid_hw *)file->private_data;
	struct cam_hw_soc_info                *soc_info;

	if (!csid_hw)
		return -EINVAL;

	soc_info = &csid_hw->hw_info->soc_info;
	buf_size = min(count, (sizeof(buf)-1));
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;
	buf[buf_size] = 0;

	csid_hw->reg_val = simple_strtoul(start, &start, 16);
	cam_io_w_mb(csid_hw->reg_val, soc_info->reg_map[0].mem_base + csid_hw->offset);
	CAM_DBG(CAM_ISP, "register value = 0x%x", csid_hw->reg_val);

	return buf_size;
}

static const struct file_operations ais_ife_csid_debug_reg = {
	.open = simple_open,
	.read = ais_ife_csid_debug_reg_read,
	.write = ais_ife_csid_debug_reg_write,
	.llseek = default_llseek,
};

static int ais_ife_csid_create_debugfs_entry(struct ais_ife_csid_hw *csid_hw)
{
	int rc = 0;
	struct dentry *dbgfileptr     = NULL;
	struct dentry *debugfs_device = NULL;
	char debugfs_name[DEBUGFS_NAME_MAX_SIZE];

	CAM_DBG(CAM_ISP, "create ais csid%d debugfs.", csid_hw->hw_intf->hw_idx);

	dbgfileptr = debugfs_lookup("camera", NULL);
	if (!dbgfileptr) {
		CAM_ERR(CAM_ISP, "csid%d: camera root debugfs dir lookup failed", csid_hw->hw_intf->hw_idx);
		rc = -ENOENT;
		goto end;
	}

	dbgfileptr = debugfs_lookup("ais_csid", dbgfileptr);
	if (!dbgfileptr) {
		CAM_ERR(CAM_ISP, "csid%d: ais-csid root debugfs dir lookup failed",
				csid_hw->hw_intf->hw_idx);
		rc = -ENOENT;
		goto end;
	}

	snprintf(debugfs_name, DEBUGFS_NAME_MAX_SIZE, "ais_csid%u",
			csid_hw->hw_intf->hw_idx);
	dbgfileptr = debugfs_create_dir(debugfs_name, dbgfileptr);
	if (!dbgfileptr) {
		CAM_ERR(CAM_ISP, "debugfs directory %s creation fail", debugfs_name);
		rc = -ENOENT;
		return 0;
	}

	debugfs_device = dbgfileptr;
	csid_hw->entry = debugfs_device;

	debugfs_create_u32("offset", 0600, debugfs_device, &csid_hw->offset);
	dbgfileptr = debugfs_create_file("register_val", 0600,
			debugfs_device, csid_hw, &ais_ife_csid_debug_reg);

end:
	return rc;
}

static void ais_ife_csid_remove_debugfs_entry(struct ais_ife_csid_hw *csid_hw)
{
	debugfs_remove_recursive(csid_hw->entry);
}

static int ais_ife_csid_ver2_wait_for_reset(
	struct ais_ife_csid_hw *csid_hw)
{
	unsigned long rem_jiffies = 0;
	int rc = 0;

	rem_jiffies = cam_common_wait_for_completion_timeout(
			&csid_hw->hw_info->hw_complete,
			msecs_to_jiffies(AIS_IFE_CSID_RESET_TIMEOUT_MS));

	if (rem_jiffies == 0) {
		rc = -ETIMEDOUT;
		CAM_ERR(CAM_ISP, "CSID[%u] reset timed out", csid_hw->hw_intf->hw_idx);
	} else {
		CAM_DBG(CAM_ISP,
				"CSID[%u] reset success", csid_hw->hw_intf->hw_idx);
	}

	return rc;
}

static int ais_ife_csid_global_reset(struct ais_ife_csid_hw *csid_hw)
{
	uint32_t val = 0, i;
	struct cam_ife_csid_ver2_reg_info *csid_reg;
	struct cam_hw_soc_info                *soc_info;
	void __iomem *mem_base;
	int rc = 0;

	csid_reg = (struct cam_ife_csid_ver2_reg_info *)
		csid_hw->csid_info->csid_reg;
	soc_info = &csid_hw->hw_info->soc_info;
	mem_base = soc_info->reg_map[0].mem_base;

	if (csid_hw->hw_info->hw_state != CAM_HW_STATE_POWER_UP) {
		CAM_ERR(CAM_ISP, "CSID[%d] powered down state",
				csid_hw->hw_intf->hw_idx);
		return -EINVAL;
	}

	val = (csid_reg->cmn_reg->rst_loc_complete_csid_val <<
			csid_reg->cmn_reg->rst_location_shift_val);
	val |= (csid_reg->cmn_reg->rst_mode_immediate_val <<
			csid_reg->cmn_reg->rst_mode_shift_val);

	cam_io_w_mb(val, mem_base + csid_reg->cmn_reg->reset_cfg_addr);

	val = 0;
	val = csid_reg->cmn_reg->rst_cmd_sw_reset_complete_val;
	cam_io_w_mb(val, mem_base + csid_reg->cmn_reg->reset_cmd_addr);

	rc = ais_ife_csid_ver2_wait_for_reset(csid_hw);
	if (rc < 0) {
		CAM_ERR(CAM_ISP, "CSID:%d csid_reset fail rc = %d",
				csid_hw->hw_intf->hw_idx, rc);
		rc = -ETIMEDOUT;
	} else {
		CAM_DBG(CAM_ISP, "CSID:%d csid_reset success",
				csid_hw->hw_intf->hw_idx);
	}
	reinit_completion(&csid_hw->hw_info->hw_complete);

	usleep_range(3000, 3010);
	val = cam_io_r_mb(mem_base +
			csid_reg->csi2_reg->irq_mask_addr);
	if (val != 0)
		CAM_ERR(CAM_ISP, "CSID:%d IRQ value after reset rc = %d",
				csid_hw->hw_intf->hw_idx, val);
	csid_hw->error_irq_count = 0;

	for (i = 0 ; i < AIS_IFE_CSID_RDI_MAX; i++) {
		csid_hw->rdi_cfg[i].state = AIS_ISP_RESOURCE_STATE_AVAILABLE;
		csid_hw->rdi_cfg[i].sof_cnt = 0;
		csid_hw->rdi_cfg[i].prev_sof_hw_ts = 0;
		csid_hw->rdi_cfg[i].prev_sof_boot_ts = 0;
		csid_hw->rdi_cfg[i].measure_cfg.measure_enabled = 0;
	}

	return rc;
}

static int ais_ife_csid_restart_rdi_path(
	struct ais_ife_csid_hw          *csid_hw)
{
	uint32_t val = 0;
	struct cam_ife_csid_ver2_reg_info *csid_reg;
	struct cam_hw_soc_info                *soc_info;
	void __iomem *mem_base;
	uint32_t path = 0;
	int rc = 0;

	csid_reg = (struct cam_ife_csid_ver2_reg_info *)
		csid_hw->csid_info->csid_reg;
	soc_info = &csid_hw->hw_info->soc_info;
	mem_base = soc_info->reg_map[0].mem_base;

	for (path = AIS_IFE_PIX_PATH_RES_RDI_0; path < csid_reg->cmn_reg->num_rdis; path++) {
		if (csid_hw->rdi_cfg[path].state == AIS_ISP_RESOURCE_STATE_STREAMING) {
			csid_hw->vfe_hw->hw_ops.process_cmd(
					csid_hw->vfe_hw->hw_priv,
					AIS_VFE_CMD_RESTART_PATH,
					&path, sizeof(uint32_t));
			val = cam_io_r_mb(soc_info->reg_map[0].mem_base + csid_reg->path_reg[path]->ctrl_addr);
			val |= csid_reg->path_reg[path]->resume_frame_boundary;
			cam_io_w_mb(val, soc_info->reg_map[0].mem_base + csid_reg->path_reg[path]->ctrl_addr);
		}
	}

	return rc;
}

static int ais_ife_csid_path_reset(struct ais_ife_csid_hw *csid_hw)
{
	uint32_t val = 0;
	struct cam_ife_csid_ver2_reg_info *csid_reg;
	struct cam_hw_soc_info                *soc_info;
	void __iomem *mem_base;
	int rc = 0;

	csid_reg = (struct cam_ife_csid_ver2_reg_info *)
		csid_hw->csid_info->csid_reg;
	soc_info = &csid_hw->hw_info->soc_info;
	mem_base = soc_info->reg_map[0].mem_base;

	if (csid_hw->hw_info->hw_state != CAM_HW_STATE_POWER_UP) {
		CAM_ERR(CAM_ISP, "CSID[%d] powered down state",
				csid_hw->hw_intf->hw_idx);
		return -EINVAL;
	}

	val |= (csid_reg->cmn_reg->rst_loc_path_only_val <<
			csid_reg->cmn_reg->rst_location_shift_val);
	val |= (csid_reg->cmn_reg->rst_mode_immediate_val <<
			csid_reg->cmn_reg->rst_mode_shift_val);

	cam_io_w_mb(val, mem_base + csid_reg->cmn_reg->reset_cfg_addr);

	val = 0;
	val = csid_reg->cmn_reg->rst_cmd_hw_reset_complete_val;
	cam_io_w_mb(val, mem_base + csid_reg->cmn_reg->reset_cmd_addr);

	rc = ais_ife_csid_ver2_wait_for_reset(csid_hw);
	if (rc < 0) {
		CAM_ERR(CAM_ISP, "CSID:%d csid_reset fail rc = %d",
				csid_hw->hw_intf->hw_idx, rc);
		rc = -ETIMEDOUT;
	} else {
		CAM_DBG(CAM_ISP, "CSID:%d csid_reset success",
				csid_hw->hw_intf->hw_idx);
	}
	reinit_completion(&csid_hw->hw_info->hw_complete);

	ais_ife_csid_restart_rdi_path(csid_hw);

	return rc;
}

static int ais_ife_csid_enable_csi2(
	struct ais_ife_csid_hw          *csid_hw,
	struct ais_ife_csid_csi_info    *csi_info)
{
	int rc = 0;
	struct cam_hw_soc_info              *soc_info;
	const struct cam_ife_csid_ver2_reg_info *csid_reg;
	const struct cam_ife_csid_csi2_rx_reg_info  *csi2_reg;
	uint32_t val = 0;
	void __iomem *mem_base;
	uint32_t vc, dt;

	csid_reg = (struct cam_ife_csid_ver2_reg_info *)
		csid_hw->csid_info->csid_reg;
	csi2_reg  = csid_reg->csi2_reg;
	soc_info = &csid_hw->hw_info->soc_info;
	mem_base = soc_info->reg_map[0].mem_base;
	CAM_DBG(CAM_ISP, "CSID:%d count:%d config csi2 rx",
			csid_hw->hw_intf->hw_idx, csid_hw->csi2_cfg_cnt);

	/* overflow check before increment */
	if (csid_hw->csi2_cfg_cnt == UINT_MAX) {
		CAM_ERR(CAM_ISP, "CSID:%d Open count reached max",
				csid_hw->hw_intf->hw_idx);
		return -EINVAL;
	}

	csid_hw->csi2_cfg_cnt++;
	if (csid_hw->csi2_cfg_cnt > 1)
		return rc;

	csid_hw->csi2_rx_cfg.phy_sel = csi_info->csiphy_id + 1;
	csid_hw->csi2_rx_cfg.lane_num = csi_info->num_lanes;
	csid_hw->csi2_rx_cfg.lane_cfg = csi_info->lane_assign;
	csid_hw->csi2_rx_cfg.lane_type = csi_info->is_3Phase;

	/*Configure Rx cfg0 */
	val |= ((csid_hw->csi2_rx_cfg.lane_cfg << csi2_reg->lane_cfg_shift) |
			((csid_hw->csi2_rx_cfg.lane_num - 1) << csi2_reg->lane_num_shift) |
			(csid_hw->csi2_rx_cfg.lane_type << csi2_reg->phy_type_shift));
	val |= csid_hw->csi2_rx_cfg.phy_sel << csi2_reg->phy_num_shift;

	cam_io_w_mb(val, mem_base + csi2_reg->cfg0_addr);
	CAM_DBG(CAM_ISP, "CSID[%d] rx_cfg0: 0x%x",
			csid_hw->hw_intf->hw_idx, val);

	val = 0;
	/*Configure Rx cfg1*/
	val = 1 << csi2_reg->misr_enable_shift_val;
	val |= 1 << csi2_reg->ecc_correction_shift_en;

	if (csi_info->vcx_mode)
		val |= (1 << csi2_reg->vc_mode_shift_val);
	cam_io_w_mb(val, mem_base + csi2_reg->cfg1_addr);
	CAM_DBG(CAM_ISP, "CSID[%d] rx_cfg1: 0x%x",
			csid_hw->hw_intf->hw_idx, val);

	/*enable rx irqs*/
	val = cam_io_r_mb(mem_base + csid_reg->top_irq_reg_info->irq_reg_set->mask_reg_offset);
	val |= csid_reg->csi2_reg->top_irq_mask;
	cam_io_w_mb(val,
			mem_base + csid_reg->top_irq_reg_info->irq_reg_set->mask_reg_offset);

	val = csi2_reg->fatal_err_mask | csi2_reg->part_fatal_err_mask |
		csi2_reg->non_fatal_err_mask | (1 << 27);
	cam_io_w_mb(val, mem_base + csid_reg->rx_irq_reg_info->irq_reg_set->mask_reg_offset);

	//rx capture config
	vc = csi_info->vc;
	dt = csi_info->dt;

	/* CAM_IFE_CSID_DEBUG_ENABLE_LONG_PKT_CAPTURE */
	val |= ((1 << csid_reg->csi2_reg->capture_long_pkt_en_shift) |
			(dt << csid_reg->csi2_reg->capture_long_pkt_dt_shift) |
			(vc << csid_reg->csi2_reg->capture_long_pkt_vc_shift));
	val = ((1 << csid_reg->csi2_reg->capture_cphy_pkt_en_shift) |
			(dt << csid_reg->csi2_reg->capture_cphy_pkt_dt_shift) |
			(vc << csid_reg->csi2_reg->capture_cphy_pkt_vc_shift));

	cam_io_w_mb(val, mem_base + csid_reg->csi2_reg->capture_ctrl_addr);
	CAM_DBG(CAM_ISP, "CSID[%d] rx capture_ctrl: 0x%x", csid_hw->hw_intf->hw_idx, val);

	//rst_strobes_addr
	val = 0xF;
	cam_io_w_mb(val, mem_base + csid_reg->csi2_reg->rst_strobes_addr);

	return rc;
}

static int ais_ife_csid_disable_csi2(struct ais_ife_csid_hw *csid_hw)
{
	int rc = 0;
	const struct cam_ife_csid_ver2_reg_info *csid_reg;
	struct cam_hw_soc_info               *soc_info;

	csid_reg = (struct cam_ife_csid_ver2_reg_info *)
		csid_hw->csid_info->csid_reg;
	soc_info = &csid_hw->hw_info->soc_info;
	CAM_DBG(CAM_ISP, "CSID:%d cnt : %d Disable csi2 rx",
			csid_hw->hw_intf->hw_idx, csid_hw->csi2_cfg_cnt);

	if (csid_hw->csi2_cfg_cnt)
		csid_hw->csi2_cfg_cnt--;

	if (csid_hw->csi2_cfg_cnt)
		return 0;

	/* Disable the CSI2 rx inerrupts */
	cam_io_w_mb(0, soc_info->reg_map[0].mem_base +
			csid_reg->rx_irq_reg_info->irq_reg_set->mask_reg_offset);

	return rc;
}

static void ais_ife_csid_halt_csi2(
	struct ais_ife_csid_hw			*csid_hw)
{
	const struct cam_ife_csid_ver2_reg_info *csid_reg;
	struct cam_hw_soc_info               *soc_info;

	csid_reg = (struct cam_ife_csid_ver2_reg_info *)
		csid_hw->csid_info->csid_reg;
	soc_info = &csid_hw->hw_info->soc_info;

	/* Disable the CSI2 rx inerrupts */
	cam_io_w(0, soc_info->reg_map[0].mem_base +
		csid_reg->rx_irq_reg_info->irq_reg_set->mask_reg_offset);

	/* Reset the Rx CFG registers */
	cam_io_w(0, soc_info->reg_map[0].mem_base +
		csid_reg->csi2_reg->cfg0_addr);
	cam_io_w_mb(0, soc_info->reg_map[0].mem_base +
		csid_reg->csi2_reg->cfg1_addr);
}


static int ais_ife_csid_get_time_stamp(
	struct ais_ife_csid_hw   *csid_hw,
	void *cmd_args)
{
	struct ais_ife_rdi_get_timestamp_args      *p_timestamp;
	const struct cam_ife_csid_ver2_reg_info *csid_reg;
	struct cam_hw_soc_info                     *soc_info;
	const struct cam_ife_csid_ver2_path_reg_info *path_reg = NULL;
	uint32_t  time_32_lsb, time_32_msb, id;
	uint64_t  time_64;

	p_timestamp = (struct ais_ife_rdi_get_timestamp_args *)cmd_args;
	csid_reg = (struct cam_ife_csid_ver2_reg_info *)
		csid_hw->csid_info->csid_reg;
	soc_info = &csid_hw->hw_info->soc_info;

	id = p_timestamp->path;

	if (id >= AIS_IFE_CSID_RDI_MAX || !csid_reg->path_reg[id]) {
		CAM_DBG(CAM_ISP, "CSID:%d Invalid RDI%d",
				csid_hw->hw_intf->hw_idx, id);
		return -EINVAL;
	}

	if (csid_hw->hw_info->hw_state != CAM_HW_STATE_POWER_UP) {
		CAM_ERR(CAM_ISP, "CSID:%d Invalid dev state :%d",
				csid_hw->hw_intf->hw_idx,
				csid_hw->hw_info->hw_state);
		return -EINVAL;
	}

	path_reg = csid_reg->path_reg[id];
	time_32_lsb = cam_io_r_mb(soc_info->reg_map[0].mem_base +
			path_reg->timestamp_curr0_sof_addr);
	time_32_msb = cam_io_r_mb(soc_info->reg_map[0].mem_base +
			path_reg->timestamp_curr1_sof_addr);
	time_64 = ((uint64_t)time_32_msb << 32) | (uint64_t)time_32_lsb;
	p_timestamp->ts->cur_sof_ts = mul_u64_u32_div(time_64,
			AIS_IFE_CSID_QTIMER_MUL_FACTOR,
			AIS_IFE_CSID_QTIMER_DIV_FACTOR);

	time_32_lsb = cam_io_r_mb(soc_info->reg_map[0].mem_base +
			path_reg->timestamp_perv0_sof_addr);
	time_32_msb = cam_io_r_mb(soc_info->reg_map[0].mem_base +
			path_reg->timestamp_perv1_sof_addr);
	time_64 = ((uint64_t)time_32_msb << 32) | (uint64_t)time_32_lsb;
	p_timestamp->ts->prev_sof_ts = mul_u64_u32_div(time_64,
			AIS_IFE_CSID_QTIMER_MUL_FACTOR,
			AIS_IFE_CSID_QTIMER_DIV_FACTOR);

	csid_hw->rdi_cfg[id].prev_sof_hw_ts = p_timestamp->ts->cur_sof_ts;

	return 0;
}

static int ais_ife_csid_set_csid_debug(struct ais_ife_csid_hw   *csid_hw,
	void *cmd_args)
{
	uint32_t  *csid_debug;

	csid_debug = (uint32_t  *) cmd_args;
	csid_hw->csid_debug = *csid_debug;
	CAM_DBG(CAM_ISP, "CSID:%d set csid debug value:%d",
			csid_hw->hw_intf->hw_idx, csid_hw->csid_debug);

	return 0;
}

static int ais_ife_csid_enable_hw(struct ais_ife_csid_hw  *csid_hw)
{
	int rc = 0;
	const struct cam_ife_csid_ver2_reg_info *csid_reg = NULL;
	const struct cam_ife_csid_ver2_path_reg_info *path_reg = NULL;
	struct cam_hw_soc_info              *soc_info;
	uint32_t i, val;
	void __iomem *mem_base;

	csid_reg = (struct cam_ife_csid_ver2_reg_info *)
		csid_hw->csid_info->csid_reg;
	soc_info = &csid_hw->hw_info->soc_info;
	mem_base = soc_info->reg_map[0].mem_base;

	/* overflow check before increment */
	if (csid_hw->hw_info->open_count == UINT_MAX) {
		CAM_ERR(CAM_ISP, "CSID:%d Open count reached max",
				csid_hw->hw_intf->hw_idx);
		return -EINVAL;
	}

	/* Increment ref Count */
	csid_hw->hw_info->open_count++;
	if (csid_hw->hw_info->open_count > 1) {
		CAM_DBG(CAM_ISP, "CSID hw has already been enabled");
		return rc;
	}

	CAM_DBG(CAM_ISP, "CSID:%d init CSID HW",
			csid_hw->hw_intf->hw_idx);
	CAM_DBG(CAM_ISP, "CSID:%d init CSID HW %p",
			csid_hw->hw_intf->hw_idx, csid_hw);

	rc = ais_ife_csid_enable_soc_resources(soc_info, CAM_TURBO_VOTE);
	if (rc) {
		CAM_ERR(CAM_ISP, "CSID:%d Enable SOC failed",
				csid_hw->hw_intf->hw_idx);
		goto err;
	}

	reinit_completion(&csid_hw->hw_info->hw_complete);
	csid_hw->hw_info->hw_state = CAM_HW_STATE_POWER_UP;

	/* Reset CSID top */
	rc = ais_ife_csid_global_reset(csid_hw);
	if (rc)
		goto disable_soc;

	if (csid_reg->need_top_cfg) {
		val = cam_io_r_mb(soc_info->reg_map[1].mem_base +
				csid_reg->top_reg->io_path_cfg0_addr[csid_hw->hw_intf->hw_idx]);
		val |= 1 << csid_reg->top_reg->out_ife_en_shift_val;
		cam_io_w_mb(val, soc_info->reg_map[1].mem_base +
				csid_reg->top_reg->io_path_cfg0_addr[csid_hw->hw_intf->hw_idx]);
	}

	/* Clear IRQs */
	cam_io_w_mb(1, mem_base + csid_reg->cmn_reg->top_irq_clear_addr);

	cam_io_w_mb(csid_reg->csi2_reg->irq_mask_all,
			mem_base + csid_reg->csi2_reg->irq_clear_addr);

	path_reg = csid_reg->path_reg[AIS_IFE_PIX_PATH_RES_IPP];
	if (csid_reg->cmn_reg->num_pix)
		cam_io_w_mb(csid_reg->cmn_reg->ipp_irq_mask_all,
				mem_base + path_reg->irq_clear_addr);

	path_reg = csid_reg->path_reg[AIS_IFE_PIX_PATH_RES_PPP];
	if (csid_reg->cmn_reg->num_ppp)
		cam_io_w_mb(csid_reg->cmn_reg->ppp_irq_mask_all,
				mem_base + path_reg->irq_clear_addr);

	for (i = 0; i < csid_reg->cmn_reg->num_rdis; i++) {
		path_reg = csid_reg->path_reg[AIS_IFE_PIX_PATH_RES_RDI_0 + i];
		cam_io_w_mb(csid_reg->cmn_reg->rdi_irq_mask_all,
				mem_base + path_reg->irq_clear_addr);
	}
	cam_io_w_mb(1, mem_base + csid_reg->cmn_reg->irq_cmd_addr);

	/* Read hw version */
	val = cam_io_r_mb(mem_base + csid_reg->cmn_reg->hw_version_addr);
	CAM_DBG(CAM_ISP, "CSID:%d CSID HW version: 0x%x",
			csid_hw->hw_intf->hw_idx, val);

	/* enable top irq */
	val = cam_io_r_mb(soc_info->reg_map[0].mem_base +
			csid_reg->top_irq_reg_info->irq_reg_set->mask_reg_offset);
	val |= csid_reg->cmn_reg->top_buf_done_irq_mask;
	cam_io_w_mb(val,
			mem_base + csid_reg->top_irq_reg_info->irq_reg_set->mask_reg_offset);

	val |= csid_reg->cmn_reg->top_err_irq_mask;
	cam_io_w_mb(val,
			mem_base + csid_reg->top_irq_reg_info->irq_reg_set->mask_reg_offset);

	return 0;

disable_soc:
	ais_ife_csid_disable_soc_resources(soc_info);
	csid_hw->hw_info->hw_state = CAM_HW_STATE_POWER_DOWN;
err:
	csid_hw->hw_info->open_count--;
	return rc;
}

static int ais_ife_csid_disable_hw(struct ais_ife_csid_hw *csid_hw)
{
	int rc = -EINVAL;
	uint32_t i;
	struct cam_hw_soc_info                   *soc_info;
	const struct cam_ife_csid_ver2_reg_info *csid_reg = NULL;
	unsigned long                             flags;

	/* Check for refcount */
	if (!csid_hw->hw_info->open_count) {
		CAM_WARN(CAM_ISP, "Unbalanced disable_hw");
		return rc;
	}

	/*  Decrement ref Count */
	csid_hw->hw_info->open_count--;

	if (csid_hw->hw_info->open_count) {
		rc = 0;
		return rc;
	}

	soc_info = &csid_hw->hw_info->soc_info;
	csid_reg = (struct cam_ife_csid_ver2_reg_info *)
		csid_hw->csid_info->csid_reg;

	CAM_DBG(CAM_ISP, "CSID:%d De-init CSID HW",
			csid_hw->hw_intf->hw_idx);

	/*disable the top IRQ interrupt */
	cam_io_w_mb(0, soc_info->reg_map[0].mem_base +
			csid_reg->cmn_reg->top_irq_mask_addr);

	rc = ais_ife_csid_disable_soc_resources(soc_info);
	if (rc)
		CAM_ERR(CAM_ISP, "CSID:%d Disable CSID SOC failed",
				csid_hw->hw_intf->hw_idx);

	spin_lock_irqsave(&csid_hw->lock_state, flags);
	csid_hw->device_enabled = 0;
	spin_unlock_irqrestore(&csid_hw->lock_state, flags);
	for (i = 0; i < AIS_IFE_CSID_RDI_MAX; i++) {
		csid_hw->rdi_cfg[i].state = AIS_ISP_RESOURCE_STATE_AVAILABLE;
		csid_hw->rdi_cfg[i].sof_cnt = 0;
		csid_hw->rdi_cfg[i].prev_sof_boot_ts = 0;
		csid_hw->rdi_cfg[i].prev_sof_hw_ts = 0;
		csid_hw->rdi_cfg[i].measure_cfg.measure_enabled = 0;
	}

	csid_hw->hw_info->hw_state = CAM_HW_STATE_POWER_DOWN;
	csid_hw->error_irq_count = 0;
	csid_hw->fatal_err_detected = false;

	return rc;
}

static int ais_ife_csid_config_rdi_path(
	struct ais_ife_csid_hw          *csid_hw,
	struct ais_ife_rdi_init_args         *res)
{
	int rc = 0;
	const struct cam_ife_csid_ver2_reg_info *csid_reg;
	struct cam_hw_soc_info                   *soc_info;
	const struct cam_ife_csid_ver2_path_reg_info *path_reg = NULL;
	const struct cam_ife_csid_ver2_common_reg_info *cmn_reg = NULL;
	uint32_t  val, cfg0 = 0, cfg1 = 0, id = 0;
	struct ais_ife_csid_path_cfg           *path_cfg;
	uint32_t                               rup_aup_mask = 0;
	uint32_t  epoch_cfg;
	void __iomem *mem_base;

	soc_info = &csid_hw->hw_info->soc_info;
	csid_reg = (struct cam_ife_csid_ver2_reg_info *)
		csid_hw->csid_info->csid_reg;

	id = res->path;
	if (id >= AIS_IFE_CSID_RDI_MAX || id >= csid_reg->cmn_reg->num_rdis ||
			!csid_reg->path_reg[id]) {
		CAM_ERR(CAM_ISP, "CSID:%d RDI:%d is not supported on HW",
				csid_hw->hw_intf->hw_idx, id);
		return -EINVAL;
	}

	cmn_reg = csid_reg->cmn_reg;
	path_reg = csid_reg->path_reg[id];
	mem_base = soc_info->reg_map[0].mem_base;

	path_cfg = &csid_hw->rdi_cfg[id];
	path_cfg->vc = res->csi_cfg.vc;
	path_cfg->dt = res->csi_cfg.dt;
	path_cfg->cid = res->csi_cfg.dt_id;
	path_cfg->in_format = res->in_cfg.format;
	path_cfg->out_format = res->out_cfg.format;
	path_cfg->crop_enable = res->in_cfg.crop_enable;
	path_cfg->start_pixel = res->in_cfg.crop_left;
	path_cfg->end_pixel = res->in_cfg.crop_right;
	path_cfg->start_line = res->in_cfg.crop_top;
	path_cfg->end_line = res->in_cfg.crop_bottom - 1;
	path_cfg->decode_fmt = res->in_cfg.decode_format;
	path_cfg->plain_fmt = res->in_cfg.pack_type;
	path_cfg->init_frame_drop = res->in_cfg.init_frame_drop;
	epoch_cfg = (path_cfg->end_line - path_cfg->start_line) *
		csid_reg->cmn_reg->epoch_factor / 100;

	if (path_cfg->decode_fmt == 0xF)
		path_cfg->pix_enable = false;
	else
		path_cfg->pix_enable = true;

	/*Configure cfg0:
	 * VC
	 * DT
	 * Timestamp enable and strobe selection for v780
	 * DT_ID cobination
	 * Decode Format
	 * Frame_id_dec_en
	 * VFR en
	 * offline mode
	 */
	path_cfg->decode_fmt = 0x1;
	cfg0 = (path_cfg->vc << cmn_reg->vc_shift_val) |
		(path_cfg->dt << cmn_reg->dt_shift_val) |
		(path_cfg->cid << cmn_reg->dt_id_shift_val) |
		(path_cfg->decode_fmt << cmn_reg->decode_format_shift_val);

	if (cmn_reg->timestamp_enabled_in_cfg0)
		cfg0 |= (1 << path_reg->timestamp_en_shift_val) |
			(cmn_reg->timestamp_strobe_val <<
			 cmn_reg->timestamp_stb_sel_shift_val);

	cam_io_w_mb(cfg0, mem_base + path_reg->cfg0_addr);
	CAM_DBG(CAM_ISP, "CSID[%d] rdi%u cfg0_addr 0x%x",
			csid_hw->hw_intf->hw_idx, id, cfg0);

	/*configure cfg1 addr
	 * Crop/Drop parameters
	 * Timestamp enable and strobe selection
	 * Plain format
	 * Packing format
	 */
	cfg1 = (path_cfg->crop_enable << path_reg->crop_h_en_shift_val) |
		(path_cfg->crop_enable <<
		 path_reg->crop_v_en_shift_val);

	if (cmn_reg->drop_supported)
		cfg1 |= (1 <<
				path_reg->drop_v_en_shift_val) |
			(1 <<
			 path_reg->drop_h_en_shift_val);

	if (path_reg->mipi_pack_supported) {
		cfg1 |= 0x1 << path_reg->packing_fmt_shift_val;
	}

	cfg1 |= (path_cfg->plain_fmt << path_reg->plain_fmt_shift_val);

	if (csid_hw->csid_debug & CSID_DEBUG_ENABLE_HBI_VBI_INFO)
		cfg1 |= 1 << path_reg->format_measure_en_shift_val;

	if (!cmn_reg->timestamp_enabled_in_cfg0)
		cfg1 |= (1 << path_reg->timestamp_en_shift_val) |
			(cmn_reg->timestamp_strobe_val <<
			 cmn_reg->timestamp_stb_sel_shift_val);

	/* We use line smoothting only on RDI_0 in all usecases */
	if ((path_reg->capabilities &
				CAM_IFE_CSID_CAP_LINE_SMOOTHING_IN_RDI) &&
			(id == AIS_IFE_PIX_PATH_RES_RDI_0))
		cfg1 |= 1 << path_reg->pix_store_en_shift_val;

	cam_io_w_mb(cfg1, mem_base + path_reg->cfg1_addr);
	CAM_DBG(CAM_ISP, "CSID:%u RDI:%u cfg1:0x%x",
			csid_hw->hw_intf->hw_idx, id, cfg1);

	/* Configure crop info */
	if (path_cfg->crop_enable) {
		val = (((path_cfg->end_pixel & cmn_reg->crop_pix_end_mask) <<
					cmn_reg->crop_shift_val) |
				(path_cfg->start_pixel & cmn_reg->crop_pix_start_mask));
		cam_io_w_mb(val, mem_base + path_reg->hcrop_addr);
		CAM_DBG(CAM_ISP, "CSID:%d Horizontal crop config val: 0x%x",
				csid_hw->hw_intf->hw_idx, val);

		val = (((path_cfg->end_line & cmn_reg->crop_line_end_mask) <<
					csid_reg->cmn_reg->crop_shift_val) |
				(path_cfg->start_line & cmn_reg->crop_line_start_mask));
		cam_io_w_mb(val, mem_base + path_reg->vcrop_addr);
		CAM_DBG(CAM_ISP, "CSID:%d Vertical Crop config val: 0x%x",
				csid_hw->hw_intf->hw_idx, val);
	}

	rup_aup_mask |= path_reg->rup_aup_mask;
	cam_io_w_mb(rup_aup_mask, mem_base + cmn_reg->rup_aup_cmd_addr);
	CAM_DBG(CAM_ISP, "CSID:%u RUP_AUP_MUP: 0x%x",
			csid_hw->hw_intf->hw_idx, rup_aup_mask);

	if (path_reg->overflow_ctrl_en) {
		val = path_reg->overflow_ctrl_en |
			path_reg->overflow_ctrl_mode_val;
		cam_io_w_mb(val, mem_base +
				path_reg->err_recovery_cfg0_addr);
	}

	if (csid_hw->csid_debug & CSID_DEBUG_ENABLE_HBI_VBI_INFO) {
		val = cam_io_r_mb(mem_base +
				path_reg->format_measure_cfg0_addr);
		val |= csid_reg->cmn_reg->measure_en_hbi_vbi_cnt_mask;
		cam_io_w_mb(val, mem_base +
				path_reg->format_measure_cfg0_addr);
	}

	/*Program the camif part */
	cam_io_w_mb(epoch_cfg << path_reg->epoch0_shift_val,
			mem_base + path_reg->epoch_irq_cfg_addr);

	path_cfg->state = AIS_ISP_RESOURCE_STATE_INIT_HW;

	CAM_DBG(CAM_ISP, "CSID:%d RDI%d configured vc:%d dt:%d",
			csid_hw->hw_intf->hw_idx, id, path_cfg->vc, path_cfg->dt);
	return rc;
}

static int ais_ife_csid_enable_rdi_path(
	struct ais_ife_csid_hw          *csid_hw,
	struct ais_ife_rdi_start_args   *start_cmd)
{
	const struct cam_ife_csid_ver2_reg_info      *csid_reg;
	struct cam_hw_soc_info                    *soc_info;
	const struct cam_ife_csid_ver2_path_reg_info *path_reg = NULL;
	struct ais_ife_csid_path_cfg              *path_data;
	uint32_t id, val;

	if (start_cmd->path >= AIS_IFE_CSID_RDI_MAX) {
		CAM_ERR(CAM_ISP, "RDI:%d path is not supported", start_cmd->path);
		return -EINVAL;
	}

	id = start_cmd->path;
	csid_reg = (struct cam_ife_csid_ver2_reg_info *)
		csid_hw->csid_info->csid_reg;
	soc_info = &csid_hw->hw_info->soc_info;
	path_reg = csid_reg->path_reg[id];
	path_data = &csid_hw->rdi_cfg[id];
	path_data->sof_cnt = 0;

	/* Enable the required RDI interrupts */
	val = cam_io_r_mb(soc_info->reg_map[0].mem_base +
			csid_reg->top_irq_reg_info->irq_reg_set->mask_reg_offset);
	val |= path_reg->top_irq_mask;
	cam_io_w_mb(val,
			soc_info->reg_map[0].mem_base
			+ csid_reg->top_irq_reg_info->irq_reg_set->mask_reg_offset);

	val = path_reg->rup_irq_mask
		| path_reg->sof_irq_mask | path_reg->eof_irq_mask
		| path_reg->epoch0_irq_mask;

	if ((csid_hw->csid_debug & CSID_DEBUG_ENABLE_SOF_IRQ) ||
			(path_data->init_frame_drop))
		val |= CSID_PATH_INFO_INPUT_SOF;

	if (csid_hw->csid_debug & CSID_DEBUG_ENABLE_EOF_IRQ)
		val |= CSID_PATH_INFO_INPUT_EOF;

	val |= path_reg->fatal_err_mask | path_reg->non_fatal_err_mask | 0x33F;
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base +
			csid_reg->path_irq_reg_info[id]->irq_reg_set->mask_reg_offset);

	/* Enable the RDI path */
	val = cam_io_r_mb(soc_info->reg_map[0].mem_base + path_reg->cfg0_addr);
	val |= (1 << csid_reg->cmn_reg->path_en_shift_val);
	cam_io_w_mb(val, soc_info->reg_map[0].mem_base + path_reg->cfg0_addr);

	/*resume at frame boundary */
	if (!path_data->init_frame_drop) {
		CAM_DBG(CAM_ISP, "Start RDI:%d path", id);
		val = path_reg->resume_frame_boundary;
		val |= path_reg->start_mode_global << path_reg->start_mode_shift;
		cam_io_w_mb(val, soc_info->reg_map[0].mem_base + path_reg->ctrl_addr);
	}

	path_data->state = AIS_ISP_RESOURCE_STATE_STREAMING;

	return 0;
}

static int ais_ife_csid_get_hw_caps(void *hw_priv,
	void *get_hw_cap_args, uint32_t arg_size)
{
	int rc = 0;
	struct ais_ife_csid_hw_caps           *hw_caps;
	struct ais_ife_csid_hw                *csid_hw;
	struct cam_hw_info                    *csid_hw_info;
	const struct cam_ife_csid_ver2_reg_info      *csid_reg;

	if (!hw_priv || !get_hw_cap_args) {
		CAM_ERR(CAM_ISP, "CSID: Invalid args");
		return -EINVAL;
	}

	csid_hw_info = (struct cam_hw_info  *)hw_priv;
	csid_hw = (struct ais_ife_csid_hw   *)csid_hw_info->core_info;
	hw_caps = (struct ais_ife_csid_hw_caps *) get_hw_cap_args;
	csid_reg = (struct cam_ife_csid_ver2_reg_info *)
		csid_hw->csid_info->csid_reg;

	hw_caps->num_rdis = csid_reg->cmn_reg->num_rdis;
	hw_caps->num_pix = csid_reg->cmn_reg->num_pix;
	hw_caps->num_ppp = csid_reg->cmn_reg->num_ppp;
	hw_caps->major_version = csid_reg->cmn_reg->major_version;
	hw_caps->minor_version = csid_reg->cmn_reg->minor_version;
	hw_caps->version_incr = csid_reg->cmn_reg->version_incr;

	CAM_DBG(CAM_ISP,
			"CSID:%d No rdis:%d, no pix:%d, major:%d minor:%d ver :%d",
			csid_hw->hw_intf->hw_idx, hw_caps->num_rdis,
			hw_caps->num_pix, hw_caps->major_version,
			hw_caps->minor_version, hw_caps->version_incr);

	return rc;
}

static int ais_ife_csid_force_reset(void *hw_priv,
	void *reset_args, uint32_t arg_size)
{
	struct ais_ife_csid_hw          *csid_hw;
	struct cam_hw_info              *csid_hw_info;
	int rc = 0;

	if (!hw_priv) {
		CAM_ERR(CAM_ISP, "CSID:Invalid args");
		return -EINVAL;
	}

	csid_hw_info = (struct cam_hw_info  *)hw_priv;
	csid_hw = (struct ais_ife_csid_hw   *)csid_hw_info->core_info;

	mutex_lock(&csid_hw->hw_info->hw_mutex);

	/* Disable CSID HW if necessary */
	if (csid_hw_info->open_count) {
		csid_hw_info->open_count = 1;

		CAM_DBG(CAM_ISP, "Disabling CSID Hw");
		rc = ais_ife_csid_disable_hw(csid_hw);
	}

	mutex_unlock(&csid_hw->hw_info->hw_mutex);

	CAM_INFO(CAM_ISP, "Exit (%d)", rc);

	return rc;
}

static int ais_ife_csid_release(void *hw_priv,
	void *release_args, uint32_t arg_size)
{
	int rc = 0;
	struct ais_ife_csid_hw                 *csid_hw;
	struct cam_hw_info                     *csid_hw_info;
	struct ais_ife_rdi_deinit_args         *rdi_deinit;
	struct ais_ife_csid_path_cfg              *path_cfg;

	if (!hw_priv || !release_args ||
			(arg_size != sizeof(struct ais_ife_rdi_deinit_args))) {
		CAM_ERR(CAM_ISP, "CSID:Invalid arguments");
		return -EINVAL;
	}

	CAM_DBG(CAM_ISP, "Enter");
	rdi_deinit = (struct ais_ife_rdi_deinit_args *)release_args;
	csid_hw_info = (struct cam_hw_info  *)hw_priv;
	csid_hw = (struct ais_ife_csid_hw   *)csid_hw_info->core_info;
	path_cfg = &csid_hw->rdi_cfg[rdi_deinit->path];

	mutex_lock(&csid_hw->hw_info->hw_mutex);
	if (rdi_deinit->path >= AIS_IFE_CSID_RDI_MAX) {
		CAM_ERR(CAM_ISP, "CSID:%d Invalid path:%d",
				csid_hw->hw_intf->hw_idx, rdi_deinit->path);
		rc = -EINVAL;
		goto end;
	}

	if (csid_hw->rdi_cfg[rdi_deinit->path].state <
			AIS_ISP_RESOURCE_STATE_INIT_HW) {
		CAM_ERR(CAM_ISP,
				"CSID:%d path:%d Invalid state %d",
				csid_hw->hw_intf->hw_idx,
				rdi_deinit->path,
				csid_hw->rdi_cfg[rdi_deinit->path].state);
		rc = -EINVAL;
		goto end;
	}

	CAM_DBG(CAM_ISP, "De-Init RDI Path: %d", rdi_deinit->path);
	path_cfg->state = AIS_ISP_RESOURCE_STATE_AVAILABLE;

	CAM_DBG(CAM_ISP, "De-Init ife_csid");
	rc |= ais_ife_csid_disable_csi2(csid_hw);

	CAM_DBG(CAM_ISP, "Exit");

end:
	mutex_unlock(&csid_hw->hw_info->hw_mutex);

	return rc;
}

int ais_ife_csid_init_hw(void *hw_priv,
	void *init_args, uint32_t arg_size)
{
	int rc = 0;
	struct ais_ife_csid_hw                 *csid_hw;
	struct cam_hw_info                     *csid_hw_info;

	if (!hw_priv) {
		CAM_ERR(CAM_ISP, "CSID: Invalid args");
		return -EINVAL;
	}

	csid_hw_info = (struct cam_hw_info  *)hw_priv;
	csid_hw = (struct ais_ife_csid_hw   *)csid_hw_info->core_info;

	mutex_lock(&csid_hw->hw_info->hw_mutex);

	/* Initialize the csid hardware */
	rc = ais_ife_csid_enable_hw(csid_hw);

	mutex_unlock(&csid_hw->hw_info->hw_mutex);

	CAM_DBG(CAM_ISP, "Exit (%d)", rc);

	return rc;
}

int ais_ife_csid_deinit_hw(void *hw_priv,
	void *deinit_args, uint32_t arg_size)
{
	int rc = 0;
	struct ais_ife_csid_hw                 *csid_hw;
	struct cam_hw_info                     *csid_hw_info;
	struct ais_ife_rdi_deinit_args         *deinit;

	if (!hw_priv) {
		CAM_ERR(CAM_ISP, "CSID:Invalid arguments");
		return -EINVAL;
	}

	CAM_DBG(CAM_ISP, "Enter");

	csid_hw_info = (struct cam_hw_info  *)hw_priv;
	csid_hw = (struct ais_ife_csid_hw   *)csid_hw_info->core_info;
	deinit = (struct ais_ife_rdi_deinit_args *)deinit_args;

	mutex_lock(&csid_hw->hw_info->hw_mutex);

	ais_ife_csid_path_reset(csid_hw);

	/* Disable CSID HW */
	CAM_DBG(CAM_ISP, "Disabling CSID Hw");
	rc = ais_ife_csid_disable_hw(csid_hw);

	mutex_unlock(&csid_hw->hw_info->hw_mutex);

	CAM_DBG(CAM_ISP, "Exit");

	return rc;
}

static int ais_ife_csid_reserve(void *hw_priv,
	void *reserve_args, uint32_t arg_size)
{
	int rc = 0;
	struct ais_ife_csid_hw                 *csid_hw;
	struct cam_hw_info                     *csid_hw_info;
	struct ais_ife_rdi_init_args            *rdi_cfg;
	unsigned long                           flags;

	if (!hw_priv || !reserve_args ||
			(arg_size != sizeof(struct ais_ife_rdi_init_args))) {
		CAM_ERR(CAM_ISP, "CSID: Invalid args");
		return -EINVAL;
	}

	csid_hw_info = (struct cam_hw_info  *)hw_priv;
	csid_hw = (struct ais_ife_csid_hw   *)csid_hw_info->core_info;
	rdi_cfg = (struct ais_ife_rdi_init_args *)reserve_args;

	mutex_lock(&csid_hw->hw_info->hw_mutex);
	if (rdi_cfg->path >= AIS_IFE_CSID_RDI_MAX) {
		CAM_ERR(CAM_ISP, "CSID:%d Invalid RDI%d",
				csid_hw->hw_intf->hw_idx, rdi_cfg->path);
		rc = -EINVAL;
		goto end;
	}

	if (csid_hw->rdi_cfg[rdi_cfg->path].state !=
			AIS_ISP_RESOURCE_STATE_AVAILABLE) {
		CAM_ERR(CAM_ISP,
				"CSID:%d RDI%d Invalid state %d",
				csid_hw->hw_intf->hw_idx,
				rdi_cfg->path, csid_hw->rdi_cfg[rdi_cfg->path].state);
		rc = -EINVAL;
		goto end;
	}

	CAM_DBG(CAM_ISP, "CSID:%d res path :%d",
			csid_hw->hw_intf->hw_idx, rdi_cfg->path);

	rc = ais_ife_csid_enable_csi2(csid_hw, &rdi_cfg->csi_cfg);
	if (rc)
		goto end;

	if (csid_hw->device_enabled == 0) {
		if (rc < 0) {
			CAM_ERR(CAM_ISP, "CSID: Failed in SW reset");
			goto disable_csi2;
		} else {
			CAM_DBG(CAM_ISP, "CSID: SW reset Successful");
			spin_lock_irqsave(&csid_hw->lock_state, flags);
			csid_hw->device_enabled = 1;
			spin_unlock_irqrestore(&csid_hw->lock_state, flags);
		}
	}

	rc = ais_ife_csid_config_rdi_path(csid_hw, rdi_cfg);
	if (rc)
		goto disable_csi2;

disable_csi2:
	if (rc)
		ais_ife_csid_disable_csi2(csid_hw);
end:
	mutex_unlock(&csid_hw->hw_info->hw_mutex);

	return rc;
}

static int ais_ife_csid_start(void *hw_priv, void *start_args,
	uint32_t arg_size)
{
	int rc = 0;
	struct ais_ife_csid_hw                 *csid_hw;
	struct cam_hw_soc_info                *soc_info;
	struct cam_hw_info                     *csid_hw_info;
	const struct cam_ife_csid_ver2_reg_info     *csid_reg;
	struct ais_ife_rdi_start_args          *start_cmd;

	if (!hw_priv || !start_args ||
			(arg_size != sizeof(struct ais_ife_rdi_start_args))) {
		CAM_ERR(CAM_ISP, "CSID: Invalid args");
		return -EINVAL;
	}

	csid_hw_info = (struct cam_hw_info  *)hw_priv;
	csid_hw = (struct ais_ife_csid_hw   *)csid_hw_info->core_info;
	soc_info = &csid_hw->hw_info->soc_info;
	csid_reg = (struct cam_ife_csid_ver2_reg_info *)
		csid_hw->csid_info->csid_reg;
	start_cmd = (struct ais_ife_rdi_start_args *)start_args;

	if (start_cmd->path >= csid_reg->cmn_reg->num_rdis ||
			!csid_reg->path_reg[start_cmd->path]) {
		CAM_ERR(CAM_ISP, "CSID:%d RDI:%d is not supported on HW",
				csid_hw->hw_intf->hw_idx, start_cmd->path);
		rc = -EINVAL;
		goto end;
	}

	/* Reset sof irq debug fields */
	csid_hw->sof_irq_triggered = false;
	csid_hw->irq_debug_cnt = 0;

	CAM_DBG(CAM_ISP, "CSID:%d res_id:%d",
			csid_hw->hw_intf->hw_idx, start_cmd->path);

	rc = ais_ife_csid_enable_rdi_path(csid_hw, start_cmd);

	/* enable buffer done irqs */
	cam_io_w_mb(0xFFFFFFFF,
			soc_info->reg_map[0].mem_base +
			csid_reg->buf_done_irq_reg_info->irq_reg_set[0].mask_reg_offset);

	cam_io_w_mb(1,
			soc_info->reg_map[0].mem_base +
			csid_reg->cmn_reg->global_cmd_addr);
	CAM_DBG(CAM_ISP, "CSID[%u] global start set",
			csid_hw->hw_intf->hw_idx);
end:
	return rc;
}

static int ais_ife_csid_stop(void *hw_priv,
	void *stop_args, uint32_t arg_size)
{
	int rc = 0;
	struct ais_ife_csid_hw               *csid_hw;
	struct cam_hw_info                   *csid_hw_info;
	const struct cam_ife_csid_ver2_reg_info     *csid_reg;
	struct ais_ife_rdi_stop_args         *stop_cmd;

	if (!hw_priv || !stop_args ||
			(arg_size != sizeof(struct ais_ife_rdi_stop_args))) {
		CAM_ERR(CAM_ISP, "CSID: Invalid args");
		return -EINVAL;
	}

	csid_hw_info = (struct cam_hw_info  *)hw_priv;
	csid_hw = (struct ais_ife_csid_hw   *)csid_hw_info->core_info;
	csid_reg = (struct cam_ife_csid_ver2_reg_info *)
		csid_hw->csid_info->csid_reg;
	stop_cmd = (struct ais_ife_rdi_stop_args  *) stop_args;

	if (stop_cmd->path >= csid_reg->cmn_reg->num_rdis ||
			!csid_reg->path_reg[stop_cmd->path]) {
		CAM_ERR(CAM_ISP, "CSID:%d RDI:%d is not supported on HW",
				csid_hw->hw_intf->hw_idx, stop_cmd->path);
		rc = -EINVAL;
		goto end;
	}

	CAM_DBG(CAM_ISP, "CSID:%d RDI %d",
			csid_hw->hw_intf->hw_idx,
			stop_cmd->path);

	if (csid_hw->error_irq_count > 0) {
		CAM_DBG(CAM_ISP, "CSID:%d RDI:%d error_irq_count:%d",
				csid_hw->hw_intf->hw_idx, stop_cmd->path, csid_hw->error_irq_count);
		csid_hw->error_irq_count--;
	}

end:
	CAM_DBG(CAM_ISP,  "Exit (%d)", rc);

	return rc;
}

static int ais_ife_csid_read(void *hw_priv,
	void *read_args, uint32_t arg_size)
{
	CAM_ERR(CAM_ISP, "CSID: un supported");

	return -EINVAL;
}

static int ais_ife_csid_write(void *hw_priv,
	void *write_args, uint32_t arg_size)
{
	CAM_ERR(CAM_ISP, "CSID: un supported");
	return -EINVAL;
}

static int ais_ife_csid_sof_irq_debug(
	struct ais_ife_csid_hw *csid_hw, void *cmd_args)
{
	bool sof_irq_enable = false;
	const struct ais_ife_csid_reg_offset    *csid_reg;
	struct cam_hw_soc_info                  *soc_info;

	csid_reg = csid_hw->csid_info->csid_reg;
	soc_info = &csid_hw->hw_info->soc_info;

	if (*((uint32_t *)cmd_args) == 1)
		sof_irq_enable = true;

	if (csid_hw->hw_info->hw_state ==
			CAM_HW_STATE_POWER_DOWN) {
		CAM_WARN(CAM_ISP,
				"CSID powered down unable to %s sof irq",
				(sof_irq_enable == true) ? "enable" : "disable");
		return 0;
	}

	if (sof_irq_enable) {
		csid_hw->csid_debug |= CSID_DEBUG_ENABLE_SOF_IRQ;
		csid_hw->sof_irq_triggered = true;
	} else {
		csid_hw->csid_debug &= ~CSID_DEBUG_ENABLE_SOF_IRQ;
		csid_hw->sof_irq_triggered = false;
	}

	CAM_INFO(CAM_ISP, "SOF freeze: CSID SOF irq %s",
			(sof_irq_enable == true) ? "enabled" : "disabled");

	return 0;
}

static int ais_ife_csid_get_total_pkts(
	struct ais_ife_csid_hw *csid_hw, void *cmd_args)
{
	struct ais_ife_diag_info                       *ife_diag;

	struct cam_hw_soc_info                         *soc_info;
	const struct cam_ife_csid_ver2_reg_info        *csid_reg;

	if (csid_hw->hw_info->hw_state != CAM_HW_STATE_POWER_UP) {
		CAM_ERR(CAM_ISP, "CSID:%d Invalid hw state :%d",
				csid_hw->hw_intf->hw_idx,
				csid_hw->hw_info->hw_state);
		return -EINVAL;
	}

	csid_reg = csid_hw->csid_info->csid_reg;
	soc_info = &csid_hw->hw_info->soc_info;

	ife_diag = (struct ais_ife_diag_info *)cmd_args;

	ife_diag->pkts_rcvd = cam_io_r_mb(soc_info->reg_map[0].mem_base +
			csid_reg->csi2_reg->total_pkts_rcvd_addr);

	return 0;
}

static int ais_ife_csid_process_cmd(void *hw_priv,
	uint32_t cmd_type, void *cmd_args, uint32_t arg_size)
{
	int rc = 0;
	struct ais_ife_csid_hw               *csid_hw;
	struct cam_hw_info                   *csid_hw_info;

	if (!hw_priv || !cmd_args) {
		CAM_ERR(CAM_ISP, "CSID: Invalid arguments");
		return -EINVAL;
	}

	csid_hw_info = (struct cam_hw_info  *)hw_priv;
	csid_hw = (struct ais_ife_csid_hw   *)csid_hw_info->core_info;

	switch (cmd_type) {
		case AIS_IFE_CSID_CMD_GET_TIME_STAMP:
			rc = ais_ife_csid_get_time_stamp(csid_hw, cmd_args);
			break;
		case AIS_IFE_CSID_SET_CSID_DEBUG:
			rc = ais_ife_csid_set_csid_debug(csid_hw, cmd_args);
			break;
		case AIS_IFE_CSID_SOF_IRQ_DEBUG:
			//rc = ais_ife_csid_sof_irq_debug(csid_hw, cmd_args);
			break;
		case AIS_ISP_HW_CMD_CSID_CLOCK_UPDATE:
			//rc = ais_ife_csid_set_csid_clock(csid_hw, cmd_args);
			break;
		case AIS_ISP_HW_CMD_DUMP_HW:
			//rc = ais_ife_csid_dump_hw(csid_hw, cmd_args);
			break;
		case AIS_IFE_CSID_CMD_DIAG_INFO:
			rc = ais_ife_csid_get_total_pkts(csid_hw, cmd_args);
			break;
		default:
			CAM_ERR(CAM_ISP, "CSID:%d unsupported cmd:%d",
					csid_hw->hw_intf->hw_idx, cmd_type);
			rc = -EINVAL;
			break;
	}

	return rc;
}

static void ais_process_workq_apply_req_worker(struct work_struct *w)
{
	cam_req_mgr_process_workq(w);
}

static int ais_csid_event_dispatch_process(void *priv, void *data)
{
	struct ais_ife_event_data evt_payload = {};
	struct ais_ife_csid_hw *csid_hw;
	struct ais_csid_hw_work_data *work_data;
	int rc = 0;

	csid_hw = (struct ais_ife_csid_hw *)priv;
	if (!csid_hw) {
		CAM_ERR(CAM_ISP, "Invalid parameters");
		return -EINVAL;
	}
	if (!csid_hw->event_cb) {
		CAM_ERR(CAM_ISP, "CSID%d Error Cb not registered",
				csid_hw->hw_intf->hw_idx);
		return -EINVAL;
	}
	work_data = (struct ais_csid_hw_work_data *)data;

	CAM_ERR_RATE_LIMIT(CAM_ISP, "CSID%d [%d %d] TOP:0x%x RX:0x%x",
			csid_hw->hw_intf->hw_idx,
			work_data->evt_type,
			csid_hw->csi2_rx_cfg.phy_sel,
			work_data->irq_status[CSID_IRQ_STATUS_TOP],
			work_data->irq_status[CSID_IRQ_STATUS_RX]);

	CAM_ERR_RATE_LIMIT(CAM_ISP, " RDIs 0x%x 0x%x 0x%x 0x%x",
			work_data->irq_status[CSID_IRQ_STATUS_RDI0],
			work_data->irq_status[CSID_IRQ_STATUS_RDI1],
			work_data->irq_status[CSID_IRQ_STATUS_RDI2],
			work_data->irq_status[CSID_IRQ_STATUS_RDI3]);

	evt_payload.msg.idx = csid_hw->hw_intf->hw_idx;
	evt_payload.msg.boot_ts = work_data->timestamp;
	evt_payload.msg.path = 0xF;
	evt_payload.u.err_msg.reserved =
		work_data->irq_status[CSID_IRQ_STATUS_RX];

	switch (work_data->evt_type) {
		case AIS_IFE_MSG_CSID_ERROR:
			if (csid_hw->fatal_err_detected)
				break;
			csid_hw->fatal_err_detected = true;

			evt_payload.msg.type = AIS_IFE_MSG_CSID_ERROR;

			rc = csid_hw->event_cb(csid_hw->event_cb_priv, &evt_payload);
			break;

		case AIS_IFE_MSG_CSID_WARNING:
			break;
		default:
			CAM_DBG(CAM_ISP, "CSID[%d] invalid error type %d",
					csid_hw->hw_intf->hw_idx,
					work_data->evt_type);
			break;
	}
	return rc;
}

static int ais_csid_dispatch_irq(struct ais_ife_csid_hw *csid_hw,
	int evt_type, uint32_t *irq_status, uint64_t timestamp)
{
	struct crm_workq_task *task;
	struct ais_csid_hw_work_data *work_data;
	int rc = 0;
	int i;

	CAM_DBG(CAM_ISP, "CSID[%d] error %d",
			csid_hw->hw_intf->hw_idx, evt_type);

	task = cam_req_mgr_workq_get_task(csid_hw->work);
	if (!task) {
		CAM_ERR(CAM_ISP, "Can not get task for worker");
		return -ENOMEM;
	}
	work_data = (struct ais_csid_hw_work_data *)task->payload;
	work_data->evt_type = evt_type;
	work_data->timestamp = timestamp;
	for (i = 0; i < CSID_IRQ_STATUS_MAX; i++)
		work_data->irq_status[i] = irq_status[i];

	task->process_cb = ais_csid_event_dispatch_process;
	rc = cam_req_mgr_workq_enqueue_task(task, csid_hw,
			CRM_TASK_PRIORITY_0);

	return rc;
}

static irqreturn_t ais_ife_csid_irq(int irq_num, void *data)
{
	struct ais_ife_csid_hw                         *csid_hw;
	struct cam_hw_soc_info                         *soc_info;
	const struct cam_ife_csid_ver2_reg_info        *csid_reg;
	const struct cam_ife_csid_csi2_rx_reg_info  *csi2_reg;
	struct ais_ife_csid_path_cfg                   *path_data;
	const struct cam_ife_csid_ver2_path_reg_info *path_reg = NULL;
	uint32_t i;
	uint32_t val, val2;
	uint32_t warn_cnt = 0;
	bool fatal_err_detected = false;
	uint32_t sof_irq_debug_en = 0;
	unsigned long flags;
	uint32_t irq_status[CSID_IRQ_STATUS_MAX] = {};
	struct timespec64 ts;
	uint32_t rup_aup_mask = 0;

	if (!data) {
		CAM_ERR(CAM_ISP, "CSID: Invalid arguments");
		return IRQ_HANDLED;
	}

	ktime_get_boottime_ts64(&ts);

	csid_hw = (struct ais_ife_csid_hw *)data;
	CAM_DBG(CAM_ISP, "CSID %d IRQ Handling", csid_hw->hw_intf->hw_idx);

	soc_info = &csid_hw->hw_info->soc_info;
	csid_reg = (struct cam_ife_csid_ver2_reg_info *)
		csid_hw->csid_info->csid_reg;
	csi2_reg = csid_reg->csi2_reg;
	path_reg = csid_reg->path_reg[0];

	/* read */
	irq_status[CSID_IRQ_STATUS_TOP] =
		cam_io_r_mb(soc_info->reg_map[0].mem_base +
				csid_reg->top_irq_reg_info->irq_reg_set->status_reg_offset);

	irq_status[CSID_IRQ_STATUS_RX] =
		cam_io_r_mb(soc_info->reg_map[0].mem_base +
				csid_reg->rx_irq_reg_info->irq_reg_set->status_reg_offset);

	irq_status[CSID_IRQ_STATUS_BUFDONE] =
		cam_io_r_mb(soc_info->reg_map[0].mem_base +
				csid_reg->buf_done_irq_reg_info->irq_reg_set->status_reg_offset);

	if (csid_reg->cmn_reg->num_pix)
		irq_status[CSID_IRQ_STATUS_IPP] =
			cam_io_r_mb(soc_info->reg_map[0].mem_base +
					csid_reg->path_irq_reg_info[AIS_IFE_PIX_PATH_RES_IPP]->
					irq_reg_set->status_reg_offset);

	if (csid_reg->cmn_reg->num_ppp)
		irq_status[CSID_IRQ_STATUS_PPP] =
			cam_io_r_mb(soc_info->reg_map[0].mem_base +
					csid_reg->path_irq_reg_info[AIS_IFE_PIX_PATH_RES_PPP]->
					irq_reg_set->status_reg_offset);

	for (i = 0; i < csid_reg->cmn_reg->num_rdis; i++) {
		irq_status[CSID_IRQ_STATUS_RDI0 + i] =
			cam_io_r_mb(soc_info->reg_map[0].mem_base +
					csid_reg->path_irq_reg_info[AIS_IFE_PIX_PATH_RES_RDI_0 + i]->
					irq_reg_set->status_reg_offset);
	}

	/* clear */
	cam_io_w_mb(irq_status[CSID_IRQ_STATUS_TOP],
			soc_info->reg_map[0].mem_base +
			csid_reg->top_irq_reg_info->irq_reg_set->clear_reg_offset);
	cam_io_w_mb(irq_status[CSID_IRQ_STATUS_RX],
			soc_info->reg_map[0].mem_base +
			csid_reg->rx_irq_reg_info->irq_reg_set->clear_reg_offset);
	cam_io_w_mb(irq_status[CSID_IRQ_STATUS_BUFDONE],
			soc_info->reg_map[0].mem_base +
			csid_reg->buf_done_irq_reg_info->irq_reg_set->clear_reg_offset);

	if (csid_reg->cmn_reg->num_pix)
		cam_io_w_mb(irq_status[CSID_IRQ_STATUS_IPP],
				soc_info->reg_map[0].mem_base +
				csid_reg->path_irq_reg_info[AIS_IFE_PIX_PATH_RES_IPP]->
				irq_reg_set->clear_reg_offset);

	if (csid_reg->cmn_reg->num_ppp)
		cam_io_w_mb(irq_status[CSID_IRQ_STATUS_PPP],
				soc_info->reg_map[0].mem_base +
				csid_reg->path_irq_reg_info[AIS_IFE_PIX_PATH_RES_PPP]->
				irq_reg_set->clear_reg_offset);

	for (i = 0; i < csid_reg->cmn_reg->num_rdis; i++) {
		cam_io_w_mb(irq_status[CSID_IRQ_STATUS_RDI0 + i],
				soc_info->reg_map[0].mem_base +
				csid_reg->path_irq_reg_info[AIS_IFE_PIX_PATH_RES_RDI_0 + i]->
				irq_reg_set->clear_reg_offset);
	}

	cam_io_w_mb(1, soc_info->reg_map[0].mem_base + csid_reg->cmn_reg->irq_cmd_addr);

	spin_lock_irqsave(&csid_hw->lock_state, flags);

	if (irq_status[CSID_IRQ_STATUS_TOP] & csid_reg->cmn_reg->top_reset_irq_mask) {
		CAM_DBG(CAM_ISP, "CSID %d TOP_IRQ_STATUS_0 = 0x%x",
				csid_hw->hw_intf->hw_idx, irq_status[CSID_IRQ_STATUS_TOP]);
		complete(&csid_hw->hw_info->hw_complete);
	}
	if (csid_hw->device_enabled == 1) {
		if (irq_status[CSID_IRQ_STATUS_RX] &
				CSID_CSI2_RX_ERROR_LANE0_FIFO_OVERFLOW) {
			fatal_err_detected = true;
			goto handle_fatal_error;
		}
		if (irq_status[CSID_IRQ_STATUS_RX] &
				CSID_CSI2_RX_ERROR_LANE1_FIFO_OVERFLOW) {
			fatal_err_detected = true;
			goto handle_fatal_error;
		}
		if (irq_status[CSID_IRQ_STATUS_RX] &
				CSID_CSI2_RX_ERROR_LANE2_FIFO_OVERFLOW) {
			fatal_err_detected = true;
			goto handle_fatal_error;
		}
		if (irq_status[CSID_IRQ_STATUS_RX] &
				CSID_CSI2_RX_ERROR_LANE3_FIFO_OVERFLOW) {
			fatal_err_detected = true;
			goto handle_fatal_error;
		}
		if (irq_status[CSID_IRQ_STATUS_RX] &
				CSID_CSI2_RX_ERROR_CRC) {
			fatal_err_detected = true;
			goto handle_fatal_error;
		}
		if (irq_status[CSID_IRQ_STATUS_RX] &
				CSID_CSI2_RX_ERROR_ECC) {
			fatal_err_detected = true;
			goto handle_fatal_error;
		}
		if (irq_status[CSID_IRQ_STATUS_RX] &
				CSID_CSI2_RX_ERROR_TG_FIFO_OVERFLOW) {
			fatal_err_detected = true;
			goto handle_fatal_error;
		}
		if (irq_status[CSID_IRQ_STATUS_RX] &
				CSID_CSI2_RX_ERROR_CPHY_EOT_RECEPTION) {
			warn_cnt++;
		}
		if (irq_status[CSID_IRQ_STATUS_RX] &
				CSID_CSI2_RX_ERROR_CPHY_SOT_RECEPTION) {
			warn_cnt++;
		}
		if (irq_status[CSID_IRQ_STATUS_RX] &
				CSID_CSI2_RX_ERROR_STREAM_UNDERFLOW) {
			warn_cnt++;
		}
		if (irq_status[CSID_IRQ_STATUS_RX] &
				CSID_CSI2_RX_ERROR_UNBOUNDED_FRAME) {
			warn_cnt++;
		}
	}

	csid_hw->error_irq_count += warn_cnt;

	if (csid_hw->error_irq_count >
			AIS_IFE_CSID_MAX_IRQ_ERROR_COUNT) {
		fatal_err_detected = true;
		csid_hw->error_irq_count = 0;
	} else if (warn_cnt) {
		uint64_t timestamp;

		timestamp = (uint64_t)((ts.tv_sec * 1000000000) + ts.tv_nsec);
		ais_csid_dispatch_irq(csid_hw,
				AIS_IFE_MSG_CSID_WARNING,
				irq_status, timestamp);
	}

handle_fatal_error:
	spin_unlock_irqrestore(&csid_hw->lock_state, flags);
	if (fatal_err_detected) {
		uint64_t timestamp;

		timestamp = (uint64_t)((ts.tv_sec * 1000000000) + ts.tv_nsec);
		CAM_INFO(CAM_ISP,
				"CSID: %d cnt: %d Halt csi2 rx irq_status_rx:0x%x",
				csid_hw->hw_intf->hw_idx, csid_hw->csi2_cfg_cnt,
				irq_status[CSID_IRQ_STATUS_RX]);
		ais_ife_csid_halt_csi2(csid_hw);
		ais_csid_dispatch_irq(csid_hw,
				AIS_IFE_MSG_CSID_ERROR,
				irq_status, timestamp);
	}

	if (csid_hw->csid_debug & CSID_DEBUG_ENABLE_EOT_IRQ) {
		if (irq_status[CSID_IRQ_STATUS_RX] &
				CSID_CSI2_RX_INFO_PHY_DL0_EOT_CAPTURED) {
			CAM_INFO_RATE_LIMIT(CAM_ISP,
					"CSID:%d PHY_DL0_EOT_CAPTURED",
					csid_hw->hw_intf->hw_idx);
		}
		if (irq_status[CSID_IRQ_STATUS_RX] &
				CSID_CSI2_RX_INFO_PHY_DL1_EOT_CAPTURED) {
			CAM_INFO_RATE_LIMIT(CAM_ISP,
					"CSID:%d PHY_DL1_EOT_CAPTURED",
					csid_hw->hw_intf->hw_idx);
		}
		if (irq_status[CSID_IRQ_STATUS_RX] &
				CSID_CSI2_RX_INFO_PHY_DL2_EOT_CAPTURED) {
			CAM_INFO_RATE_LIMIT(CAM_ISP,
					"CSID:%d PHY_DL2_EOT_CAPTURED",
					csid_hw->hw_intf->hw_idx);
		}
		if (irq_status[CSID_IRQ_STATUS_RX] &
				CSID_CSI2_RX_INFO_PHY_DL3_EOT_CAPTURED) {
			CAM_INFO_RATE_LIMIT(CAM_ISP,
					"CSID:%d PHY_DL3_EOT_CAPTURED",
					csid_hw->hw_intf->hw_idx);
		}
	}

	if (csid_hw->csid_debug & CSID_DEBUG_ENABLE_SOT_IRQ) {
		if (irq_status[CSID_IRQ_STATUS_RX] &
				CSID_CSI2_RX_INFO_PHY_DL0_SOT_CAPTURED) {
			CAM_INFO_RATE_LIMIT(CAM_ISP,
					"CSID:%d PHY_DL0_SOT_CAPTURED",
					csid_hw->hw_intf->hw_idx);
		}
		if (irq_status[CSID_IRQ_STATUS_RX] &
				CSID_CSI2_RX_INFO_PHY_DL1_SOT_CAPTURED) {
			CAM_INFO_RATE_LIMIT(CAM_ISP,
					"CSID:%d PHY_DL1_SOT_CAPTURED",
					csid_hw->hw_intf->hw_idx);
		}
		if (irq_status[CSID_IRQ_STATUS_RX] &
				CSID_CSI2_RX_INFO_PHY_DL2_SOT_CAPTURED) {
			CAM_INFO_RATE_LIMIT(CAM_ISP,
					"CSID:%d PHY_DL2_SOT_CAPTURED",
					csid_hw->hw_intf->hw_idx);
		}
		if (irq_status[CSID_IRQ_STATUS_RX] &
				CSID_CSI2_RX_INFO_PHY_DL3_SOT_CAPTURED) {
			CAM_INFO_RATE_LIMIT(CAM_ISP,
					"CSID:%d PHY_DL3_SOT_CAPTURED",
					csid_hw->hw_intf->hw_idx);
		}
	}

	if ((csid_hw->csid_debug & CSID_DEBUG_ENABLE_LONG_PKT_CAPTURE) &&
			(irq_status[CSID_IRQ_STATUS_RX] &
			 CSID_CSI2_RX_INFO_LONG_PKT_CAPTURED)) {
		CAM_INFO_RATE_LIMIT(CAM_ISP, "CSID:%d LONG_PKT_CAPTURED",
				csid_hw->hw_intf->hw_idx);
		val = cam_io_r_mb(soc_info->reg_map[0].mem_base +
				csi2_reg->captured_long_pkt_0_addr);
		CAM_INFO_RATE_LIMIT(CAM_ISP,
				"CSID:%d long packet VC :%d DT:%d WC:%d",
				csid_hw->hw_intf->hw_idx,
				((val & csi2_reg->vc_mask) >> csi2_reg->vc_shift),
				((val & csi2_reg->dt_mask) >> csi2_reg->dt_shift),
				((val & csi2_reg->wc_mask) >> csi2_reg->wc_shift));
		val = cam_io_r_mb(soc_info->reg_map[0].mem_base +
				csi2_reg->captured_long_pkt_1_addr);
		CAM_INFO_RATE_LIMIT(CAM_ISP, "CSID:%d long packet ECC :%d",
				csid_hw->hw_intf->hw_idx, val);
		val = cam_io_r_mb(soc_info->reg_map[0].mem_base +
				csi2_reg->captured_long_pkt_ftr_addr);
		CAM_INFO_RATE_LIMIT(CAM_ISP,
				"CSID:%d long pkt cal CRC:%d expected CRC:%d",
				csid_hw->hw_intf->hw_idx,
				((val >> csi2_reg->calc_crc_shift) & csi2_reg->calc_crc_mask),
				(val & csi2_reg->expected_crc_mask));
	}
	if ((csid_hw->csid_debug & CSID_DEBUG_ENABLE_SHORT_PKT_CAPTURE) &&
			(irq_status[CSID_IRQ_STATUS_RX] &
			 CSID_CSI2_RX_INFO_SHORT_PKT_CAPTURED)) {
		CAM_INFO_RATE_LIMIT(CAM_ISP, "CSID:%d SHORT_PKT_CAPTURED",
				csid_hw->hw_intf->hw_idx);
		val = cam_io_r_mb(soc_info->reg_map[0].mem_base +
				csi2_reg->captured_short_pkt_0_addr);
		CAM_INFO_RATE_LIMIT(CAM_ISP,
				"CSID:%d short pkt VC :%d DT:%d LC:%d",
				csid_hw->hw_intf->hw_idx,
				((val & csi2_reg->vc_mask) >> csi2_reg->vc_shift),
				((val & csi2_reg->dt_mask) >> csi2_reg->dt_shift),
				((val & csi2_reg->wc_mask) >> csi2_reg->wc_shift));
		val = cam_io_r_mb(soc_info->reg_map[0].mem_base +
				csi2_reg->captured_short_pkt_1_addr);
		CAM_INFO_RATE_LIMIT(CAM_ISP, "CSID:%d short packet ECC :%d",
				csid_hw->hw_intf->hw_idx, val);
	}

	if ((csid_hw->csid_debug & CSID_DEBUG_ENABLE_CPHY_PKT_CAPTURE) &&
			(irq_status[CSID_IRQ_STATUS_RX] &
			 CSID_CSI2_RX_INFO_CPHY_PKT_HDR_CAPTURED)) {
		CAM_INFO_RATE_LIMIT(CAM_ISP, "CSID:%d CPHY_PKT_HDR_CAPTURED",
				csid_hw->hw_intf->hw_idx);
		val = cam_io_r_mb(soc_info->reg_map[0].mem_base +
				csi2_reg->captured_cphy_pkt_hdr_addr);
		CAM_INFO_RATE_LIMIT(CAM_ISP,
				"CSID:%d cphy packet VC :%d DT:%d WC:%d",
				csid_hw->hw_intf->hw_idx,
				((val & csi2_reg->vc_mask) >> csi2_reg->vc_shift),
				((val & csi2_reg->dt_mask) >> csi2_reg->dt_shift),
				((val & csi2_reg->wc_mask) >> csi2_reg->wc_shift));
	}

	for (i = 0; i < csid_reg->cmn_reg->num_rdis; i++) {
		path_data = (struct ais_ife_csid_path_cfg *)
			&csid_hw->rdi_cfg[i];
		path_reg = csid_reg->path_reg[i];
		/*if (irq_status[CSID_IRQ_STATUS_RDI0 + i] &
		  BIT(csid_reg->cmn_reg->path_rst_done_shift_val)) {
		  complete(&csid_hw->csid_rdi_complete[i]);
		  }*/

		if ((irq_status[CSID_IRQ_STATUS_RDI0 + i] &
					CSID_PATH_INFO_INPUT_SOF) &&
				(csid_hw->csid_debug & CSID_DEBUG_ENABLE_SOF_IRQ)) {
			CAM_INFO_RATE_LIMIT(CAM_ISP,
					"CSID RDI:%d SOF received", i);
			if (csid_hw->sof_irq_triggered)
				csid_hw->irq_debug_cnt++;
		}

		if ((irq_status[CSID_IRQ_STATUS_RDI0 + i] &
					CSID_PATH_INFO_INPUT_SOF) &&
				(path_data->state ==
				 AIS_ISP_RESOURCE_STATE_STREAMING)) {
			rup_aup_mask |= path_reg->rup_aup_mask;
			cam_io_w_mb(rup_aup_mask, soc_info->reg_map[0].mem_base + csid_reg->cmn_reg->rup_aup_cmd_addr);
			CAM_DBG(CAM_ISP, "CSID:%u RUP_AUP_MUP: 0x%x",
					csid_hw->hw_intf->hw_idx, rup_aup_mask);
		}

		if ((irq_status[CSID_IRQ_STATUS_RDI0 + i] &
					CSID_PATH_INFO_INPUT_SOF) &&
				(path_data->init_frame_drop) &&
				(path_data->state ==
				 AIS_ISP_RESOURCE_STATE_STREAMING)) {
			path_data->sof_cnt++;
			CAM_DBG(CAM_ISP,
					"CSID:%d RDI:%d SOF cnt:%d init_frame_drop:%d",
					csid_hw->hw_intf->hw_idx, i,
					path_data->sof_cnt,
					path_data->init_frame_drop);
			if (path_data->sof_cnt ==
					path_data->init_frame_drop) {
				val = cam_io_r_mb(soc_info->reg_map[0].mem_base + path_reg->ctrl_addr);
				val |= path_reg->resume_frame_boundary;
				cam_io_w_mb(val, soc_info->reg_map[0].mem_base + path_reg->ctrl_addr);

				path_data->init_frame_drop = 0;

				/*if (!(csid_hw->csid_debug &
				  CSID_DEBUG_ENABLE_SOF_IRQ)) {
					  val = cam_io_r_mb(
						  soc_info->reg_map[0].mem_base +
						  csid_reg->path_irq_reg_info[i]->irq_reg_set->mask_reg_offset);
					  val &= ~(CSID_PATH_INFO_INPUT_SOF);
					  cam_io_w_mb(val,
						  soc_info->reg_map[0].mem_base +
						  csid_reg->path_irq_reg_info[i]->irq_reg_set->mask_reg_offset);
				  }*/
			}
		}

		if ((irq_status[CSID_IRQ_STATUS_RDI0 + i]  &
					CSID_PATH_INFO_INPUT_EOF) &&
				(csid_hw->csid_debug & CSID_DEBUG_ENABLE_EOF_IRQ))
			CAM_INFO_RATE_LIMIT(CAM_ISP,
					"CSID RDI:%d EOF received", i);

		if ((irq_status[CSID_IRQ_STATUS_RDI0 + i] &
					CSID_PATH_ERROR_CCIF_VIOLATION) ||
				(irq_status[CSID_IRQ_STATUS_RDI0 + i] &
				 CSID_PATH_ERROR_FIFO_OVERFLOW)) {
			CAM_ERR_RATE_LIMIT(CAM_ISP,
					"CSID:%d irq_status_rdi[%d]:0x%x",
					csid_hw->hw_intf->hw_idx, i,
					irq_status[CSID_IRQ_STATUS_RDI0 + i]);
		}
		if (irq_status[CSID_IRQ_STATUS_RDI0 + i] &
				CSID_PATH_ERROR_FIFO_OVERFLOW) {
			/* Stop RDI path immediately */
			cam_io_w_mb(AIS_CSID_HALT_IMMEDIATELY,
					soc_info->reg_map[0].mem_base +
					csid_reg->path_reg[i]->ctrl_addr);
		}

		if ((irq_status[CSID_IRQ_STATUS_RDI0 + i] &
					CSID_PATH_ERROR_PIX_COUNT) ||
				(irq_status[CSID_IRQ_STATUS_RDI0 + i] &
				 CSID_PATH_ERROR_LINE_COUNT)) {
			val = cam_io_r_mb(soc_info->reg_map[0].mem_base +
					csid_reg->path_reg[i]->format_measure0_addr);
			val2 = cam_io_r_mb(soc_info->reg_map[0].mem_base +
					csid_reg->path_reg[i]->format_measure_cfg1_addr
					);
			CAM_ERR(CAM_ISP,
					"CSID:%d irq_status_rdi[%d]:0x%x",
					csid_hw->hw_intf->hw_idx, i,
					irq_status[CSID_IRQ_STATUS_RDI0 + i]);
			CAM_ERR(CAM_ISP,
					"Expected sz 0x%x*0x%x actual sz 0x%x*0x%x",
					((val2 >>
					  csid_reg->cmn_reg->format_measure_height_shift_val) &
					 csid_reg->cmn_reg->format_measure_height_mask_val),
					val2 &
					csid_reg->cmn_reg->format_measure_width_mask_val,
					((val >>
					  csid_reg->cmn_reg->format_measure_height_shift_val) &
					 csid_reg->cmn_reg->format_measure_height_mask_val),
					val &
					csid_reg->cmn_reg->format_measure_width_mask_val);
		}
	}

	if (csid_hw->irq_debug_cnt >= AIS_CSID_IRQ_SOF_DEBUG_CNT_MAX) {
		ais_ife_csid_sof_irq_debug(csid_hw, &sof_irq_debug_en);
		csid_hw->irq_debug_cnt = 0;
	}

	if (irq_status[CSID_IRQ_STATUS_BUFDONE]) {
		struct ais_ife_proc_buf_done_args buf_done;
		buf_done.irq_status = irq_status[CSID_IRQ_STATUS_BUFDONE];

		CAM_INFO(CAM_ISP, "csid received buf done.");
		csid_hw->vfe_hw->hw_ops.process_cmd(
				csid_hw->vfe_hw->hw_priv,
				AIS_VFE_CMD_PROC_BUF_DONE,
				&buf_done, sizeof(buf_done));
	}

	CAM_DBG(CAM_ISP, "CSID %d IRQ Handling exit", csid_hw->hw_intf->hw_idx);

	return IRQ_HANDLED;
}

int ais_ife_csid_hw_probe_init(struct cam_hw_intf  *csid_hw_intf,
	uint32_t csid_idx)
{
	int rc = -EINVAL;
	uint32_t i;
	struct cam_hw_info                   *csid_hw_info;
	struct ais_ife_csid_hw               *ife_csid_hw = NULL;
	const struct cam_ife_csid_ver2_reg_info *csid_reg;
	char worker_name[128];

	if (csid_idx >= AIS_IFE_CSID_HW_RES_MAX) {
		CAM_ERR(CAM_ISP, "Invalid csid index:%d", csid_idx);
		return rc;
	}

	csid_hw_info = (struct cam_hw_info  *) csid_hw_intf->hw_priv;
	ife_csid_hw  = (struct ais_ife_csid_hw  *) csid_hw_info->core_info;
	csid_reg = (struct cam_ife_csid_ver2_reg_info *)ife_csid_hw->csid_info->csid_reg;

	ife_csid_hw->hw_intf = csid_hw_intf;
	ife_csid_hw->hw_info = csid_hw_info;

	CAM_DBG(CAM_ISP, "type %d index %d",
			ife_csid_hw->hw_intf->hw_type, csid_idx);

	ife_csid_hw->device_enabled = 0;
	ife_csid_hw->hw_info->hw_state = CAM_HW_STATE_POWER_DOWN;
	mutex_init(&ife_csid_hw->hw_info->hw_mutex);
	spin_lock_init(&ife_csid_hw->hw_info->hw_lock);
	spin_lock_init(&ife_csid_hw->lock_state);
	init_completion(&ife_csid_hw->hw_info->hw_complete);

	init_completion(&ife_csid_hw->csid_top_complete);
	init_completion(&ife_csid_hw->csid_csi2_complete);
	init_completion(&ife_csid_hw->csid_ipp_complete);
	init_completion(&ife_csid_hw->csid_ppp_complete);
	for (i = 0; i < AIS_IFE_CSID_RDI_MAX; i++)
		init_completion(&ife_csid_hw->csid_rdi_complete[i]);

	CAM_DBG(CAM_ISP, "type %d index %d, addr %p",
			ife_csid_hw->hw_intf->hw_type, csid_idx, ife_csid_hw);
	rc = ais_ife_csid_init_soc_resources(&ife_csid_hw->hw_info->soc_info,
			ais_ife_csid_irq, NULL, ife_csid_hw, false);
	if (rc < 0) {
		CAM_ERR(CAM_ISP, "CSID:%d Failed to init_soc", csid_idx);
		goto err;
	}

	CAM_DBG(CAM_ISP, "success ais_ife_csid_init_soc_resources");
	ife_csid_hw->hw_intf->hw_ops.get_hw_caps = ais_ife_csid_get_hw_caps;
	ife_csid_hw->hw_intf->hw_ops.init        = ais_ife_csid_init_hw;
	ife_csid_hw->hw_intf->hw_ops.deinit      = ais_ife_csid_deinit_hw;
	ife_csid_hw->hw_intf->hw_ops.reserve     = ais_ife_csid_reserve;
	ife_csid_hw->hw_intf->hw_ops.start       = ais_ife_csid_start;
	ife_csid_hw->hw_intf->hw_ops.release     = ais_ife_csid_release;
	ife_csid_hw->hw_intf->hw_ops.reset       = ais_ife_csid_force_reset;
	ife_csid_hw->hw_intf->hw_ops.stop        = ais_ife_csid_stop;
	ife_csid_hw->hw_intf->hw_ops.read        = ais_ife_csid_read;
	ife_csid_hw->hw_intf->hw_ops.write       = ais_ife_csid_write;
	ife_csid_hw->hw_intf->hw_ops.process_cmd = ais_ife_csid_process_cmd;

	/* Initialize the RDI resource */
	for (i = 0; i < csid_reg->cmn_reg->num_rdis;
			i++) {
		ife_csid_hw->rdi_cfg[i].state =
			AIS_ISP_RESOURCE_STATE_AVAILABLE;
	}

	ife_csid_hw->csid_debug = 0;
	ife_csid_hw->error_irq_count = 0;
	ife_csid_hw->total_size = ife_csid_hw->hw_info->soc_info.reg_map[0].size;

	scnprintf(worker_name, sizeof(worker_name),
			"csid%u_worker", ife_csid_hw->hw_intf->hw_idx);
	CAM_DBG(CAM_ISP, "Create CSID worker %s", worker_name);
	rc = cam_req_mgr_workq_create(worker_name,
			AIS_CSID_WORKQ_NUM_TASK,
			&ife_csid_hw->work, CRM_WORKQ_USAGE_IRQ, 0,
			ais_process_workq_apply_req_worker);
	if (rc) {
		CAM_ERR(CAM_ISP, "Unable to create a workq, rc=%d", rc);
		goto err_deinit_soc;
	}

	ife_csid_hw->work_data = kcalloc(AIS_CSID_WORKQ_NUM_TASK,
			sizeof(struct ais_csid_hw_work_data), GFP_KERNEL);
	for (i = 0; i < AIS_CSID_WORKQ_NUM_TASK; i++)
		ife_csid_hw->work->task.pool[i].payload =
			&ife_csid_hw->work_data[i];

	ais_ife_csid_create_debugfs_entry(ife_csid_hw);

	return 0;

err_deinit_soc:
	ais_ife_csid_deinit_soc_resources(&ife_csid_hw->hw_info->soc_info);
err:
	return rc;
	return 0;
}

int ais_ife_csid_hw_deinit(struct ais_ife_csid_hw *ife_csid_hw)
{
	int rc = 0;

	ais_ife_csid_remove_debugfs_entry(ife_csid_hw);
	if (ife_csid_hw) {
		ais_ife_csid_deinit_soc_resources(
				&ife_csid_hw->hw_info->soc_info);
		cam_req_mgr_workq_destroy(&ife_csid_hw->work);
	} else {
		CAM_ERR(CAM_ISP, "Invalid param");
		rc = -EINVAL;
	}

	return rc;
}
