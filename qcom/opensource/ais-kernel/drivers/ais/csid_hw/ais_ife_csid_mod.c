/* Copyright (c) 2017-2020, The Linux Foundation. All rights reserved.
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
#include "ais_ife_csid780.h"
#include "ais_ife_csid880.h"
#include "ais_ife_csid_mod.h"
#include "ais_ife_csid_dev.h"

#define AIS_CSID_DRV_NAME                    "ais-csid"

static struct cam_ife_csid_core_info cam_ife_csid780_hw_info = {
	.csid_reg = &cam_ife_csid_780_reg_info,
};
static struct cam_ife_csid_core_info cam_ife_csid880_hw_info = {
	.csid_reg = &cam_ife_csid_880_reg_info,
};


static const struct of_device_id ais_ife_csid_dt_match[] = {
	{
		.compatible = "qcom,ais-csid780",
		.data = &cam_ife_csid780_hw_info,
	},
	{
		.compatible = "qcom,ais-csid880",
		.data = &cam_ife_csid880_hw_info,
	},
	{}
};

MODULE_DEVICE_TABLE(of, ais_ife_csid_dt_match);

struct platform_driver ais_ife_csid_driver = {
	.probe = ais_ife_csid_probe,
	.remove = ais_ife_csid_remove,
	.driver = {
		.name = AIS_CSID_DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ais_ife_csid_dt_match,
		.suppress_bind_attrs = true,
	},
};

int ais_ife_csid_init_module(void)
{
	return platform_driver_register(&ais_ife_csid_driver);
}

void ais_ife_csid_exit_module(void)
{
	platform_driver_unregister(&ais_ife_csid_driver);
}

MODULE_DESCRIPTION("AIS IFE_csid780 driver");
MODULE_LICENSE("GPL v2");
