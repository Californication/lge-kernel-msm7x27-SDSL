/* arch/arm/mach-msm/board-swift-ats.c
 * Copyright (C) 2008 LGE, Inc.
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <mach/msm_rpcrouter.h>
#include "board-swift-ats.h"

/* Ats server definitions. */

#define ATS_APPS_APISPROG		0x30000006
#define ATS_APPS_APISVERS		0

#define ONCRPC_LGE_ATCMD_ATS_PROC 3
#define ONCRPC_LGE_ATCMD_ATS_ETA_PROC 6
#define ONCRPC_LGE_GET_FLEX_MCC_PROC 7 //LGE_UPDATE_S [irene.park@lge.com] 2010.06.04 - Get Flex MCC/MNC value
#define ONCRPC_LGE_GET_FLEX_MNC_PROC 8 //LGE_UPDATE_S [irene.park@lge.com] 2010.06.04 - Get Flex MCC/MNC value
#define ONCRPC_LGE_GET_FLEX_OPERATOR_CODE_PROC 9 ///LGE_UPDATE_S [irene.park@lge.com] 2010.06.04 - Get Flex Operator ?value
#define ONCRPC_LGE_GET_FLEX_COUNTRY_CODE_PROC 10 //LGE_UPDATE_S [irene.park@lge.com] 2010.06.04 - Get Flex Operator ?value

static int handle_ats_rpc_call(struct msm_rpc_server *server, struct rpc_request_hdr *req, unsigned len)
{
	switch (req->procedure)
	{
		case ONCRPC_LGE_ATCMD_ATS_PROC:
		case ONCRPC_LGE_ATCMD_ATS_ETA_PROC:
		case ONCRPC_LGE_GET_FLEX_MCC_PROC:
		case ONCRPC_LGE_GET_FLEX_MNC_PROC:
		case ONCRPC_LGE_GET_FLEX_OPERATOR_CODE_PROC:
		case ONCRPC_LGE_GET_FLEX_COUNTRY_CODE_PROC:
			printk(KERN_INFO"%s:ONCRPC_LGE_GET_FLEX_ %d\n", __func__,req->procedure);
			break;
		default:
			return -ENODEV;
	}

	return 0;
}

static int __devexit ats_remove(struct platform_device *pdev)
{
	return 0;
}

static int ats_probe(struct platform_device *pdev)
{
	return 0;
}
static struct platform_driver ats_driver = {
	
	.probe = ats_probe,
	.remove = __devexit_p(ats_remove),
	.suspend = NULL,
	.resume  = NULL,
	.driver = {
	.name = "swift_ats",
	.owner = THIS_MODULE,
	},
};

static struct msm_rpc_server ats_rpc_server = {
	.prog = ATS_APPS_APISPROG,
	.vers = ATS_APPS_APISVERS,
	.rpc_call = handle_ats_rpc_call,
};

static int __init lge_ats_init(void)
{
	int err;

	if((err = msm_rpc_create_server(&ats_rpc_server)) != 0) {
		printk(KERN_ERR"%s: Error during creating rpc server for ats\n", __func__);
		return err;
	}

	platform_driver_register(&ats_driver);

	return err;
}

module_init(lge_ats_init);

static void __exit ats_exit(void)
{
    platform_driver_unregister(&ats_driver);
}
module_exit(ats_exit);
MODULE_DESCRIPTION("LGE ATS driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:lge-ats");

