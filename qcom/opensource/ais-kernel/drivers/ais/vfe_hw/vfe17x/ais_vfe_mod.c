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
#include "ais_vfe_hw_intf.h"
#include "ais_vfe_core.h"
#include "ais_vfe_dev.h"
#include "ais_vfe780.h"
#include "ais_vfe_lite78x.h"
#include "ais_vfe880.h"
#include "ais_vfe_lite88x.h"
#include "ais_vfe_mod.h"

static const struct of_device_id ais_vfe_dt_match[] = {
	{
		.compatible = "qcom,ais-vfe780",
		.data = &ais_vfe780_hw_info,
	},
	{
		.compatible = "qcom,ais-vfe-lite780",
		.data = &ais_vfe_lite78x_hw_info,
	},
	{
		.compatible = "qcom,ais-vfe880",
		.data = &ais_vfe880_hw_info,
	},
	{
		.compatible = "qcom,ais-vfe-lite880",
		.data = &ais_vfe_lite88x_hw_info,
	},
	{}
};
MODULE_DEVICE_TABLE(of, ais_vfe_dt_match);

struct platform_driver ais_vfe_driver = {
	.probe = ais_vfe_probe,
	.remove = ais_vfe_remove,
	.driver = {
		.name = "ais_vfe",
		.owner = THIS_MODULE,
		.of_match_table = ais_vfe_dt_match,
		.suppress_bind_attrs = true,
	},
};

int ais_vfe_init_module(void)
{
	return platform_driver_register(&ais_vfe_driver);
}

void ais_vfe_exit_module(void)
{
	platform_driver_unregister(&ais_vfe_driver);
}

MODULE_DESCRIPTION("AIS VFE17X driver");
MODULE_LICENSE("GPL v2");
