/* Copyright (c) 2017-2018, 2020, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include "ais_ife_csid_core.h"
#include "ais_ife_csid_lite780.h"
#include "ais_ife_csid_lite880.h"
#include "ais_ife_csid_lite_mod.h"
#include "ais_ife_csid_dev.h"

#define CAM_CSID_LITE_DRV_NAME                    "ais-csid_lite"

static struct cam_ife_csid_core_info ais_ife_csid_lite_780_hw_info = {
	.csid_reg = &ais_ife_csid_lite_780_reg_info,
};
static struct cam_ife_csid_core_info ais_ife_csid_lite_880_hw_info = {
	.csid_reg = &ais_ife_csid_lite_880_reg_info,
};

static const struct of_device_id ais_ife_csid_lite_dt_match[] = {
	{
		.compatible = "qcom,ais-csid-lite780",
		.data = &ais_ife_csid_lite_780_hw_info,
	},
	{
		.compatible = "qcom,ais-csid-lite880",
		.data = &ais_ife_csid_lite_880_hw_info,
	},
	{}
};
MODULE_DEVICE_TABLE(of, ais_ife_csid_lite_dt_match);

struct platform_driver ais_ife_csid_lite_driver = {
	.probe = ais_ife_csid_probe,
	.remove = ais_ife_csid_remove,
	.driver = {
		.name = CAM_CSID_LITE_DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ais_ife_csid_lite_dt_match,
		.suppress_bind_attrs = true,
	},
};

int ais_ife_csid_lite_init_module(void)
{
	return platform_driver_register(&ais_ife_csid_lite_driver);
}

void ais_ife_csid_lite_exit_module(void)
{
	platform_driver_unregister(&ais_ife_csid_lite_driver);
}

MODULE_DESCRIPTION("CAM IFE_CSID_LITE driver");
MODULE_LICENSE("GPL v2");
