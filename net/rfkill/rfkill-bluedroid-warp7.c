/*
 *
 *
 * Copyright (C) 2009  Guiming Zhuo <gmzhuo@gmail.com>
 * Copyright (C) 2011  Antonio Ospite <ospite@studenti.unina.it>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/rfkill.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#define  RFKILL_REGULATOR_NAME  "bluedroid";


struct rfkill_bluedroid_data {
	struct rfkill *rf_kill;
	bool reg_enabled;
	int   bt_rst_gpio;

};

struct rfkill_bluedroid_platform_data {
	char *name;             /* the name for the rfkill switch */
	enum rfkill_type type;  /* the type as specified in rfkill.h */
};

struct rfkill_bluedroid_platform_data  g_platform_data;


static int rfkill_bluedroid_set_block(void *data, bool blocked)
{
	struct rfkill_bluedroid_data *rfkill_data = data;
	int ret = 0;

	if(blocked){
		gpio_set_value(rfkill_data->bt_rst_gpio,0);
		printk("[%s]  set gpio %d value 0\n", __func__, rfkill_data->bt_rst_gpio);
	}
	else{
		gpio_set_value(rfkill_data->bt_rst_gpio, 1);
		printk("[%s] set gpio %d value 1\n", __func__, rfkill_data->bt_rst_gpio);

	}

	return ret;
}

static struct rfkill_ops rfkill_bluedroid_ops = {
	.set_block = rfkill_bluedroid_set_block,
};


static int rfkill_bluedroid_probe(struct platform_device *pdev)
{
	struct rfkill_bluedroid_platform_data *pdata = pdev->dev.platform_data;
	struct rfkill_bluedroid_data *rfkill_data;
	struct rfkill *rf_kill;
	int ret = 0;
	int rst_gpio = 0;

	g_platform_data.name = RFKILL_REGULATOR_NAME;
	g_platform_data.type = RFKILL_TYPE_BLUETOOTH;

	pdata = &g_platform_data;


	if (pdata->name == NULL || pdata->type == 0) {
		dev_err(&pdev->dev, "invalid name or type in platform data\n");
		return -EINVAL;
	}

	rst_gpio = of_get_named_gpio( pdev->dev.of_node, "bt-reset", 0);

	if (!gpio_is_valid(rst_gpio)){
		dev_err(&pdev->dev, "no  reset pin available");
		return -EINVAL;

	}
	else {
		ret = devm_gpio_request_one(&pdev->dev, rst_gpio, GPIOF_OUT_INIT_HIGH,"bt reset");
		if (ret < 0) {
			dev_err(&pdev->dev, "Failed to set power pin\n");
			dev_err(&pdev->dev, "retval=%d\n", ret);
			return ret;
		}
	}

	rfkill_data = kzalloc(sizeof(*rfkill_data), GFP_KERNEL);

	if (rfkill_data == NULL) {
		ret = -ENOMEM;
		goto err_data_alloc;
	}

	rfkill_data->bt_rst_gpio = rst_gpio;

	printk("[%s] set  bt_rst_gpio: %d\n", __func__, rfkill_data->bt_rst_gpio);

	rf_kill = rfkill_alloc(pdata->name, &pdev->dev,
				pdata->type,
				&rfkill_bluedroid_ops, rfkill_data);
	if (rf_kill == NULL) {
		ret = -ENOMEM;
		goto err_rfkill_alloc;
	}


	rfkill_data->rf_kill = rf_kill;

	ret = rfkill_register(rf_kill);
	if (ret) {
		dev_err(&pdev->dev, "Cannot register rfkill device\n");
		goto err_rfkill_register;
	}

	platform_set_drvdata(pdev, rfkill_data);
	dev_info(&pdev->dev, "%s initialized\n", pdata->name);

	return 0;

err_rfkill_register:
	rfkill_destroy(rf_kill);
err_rfkill_alloc:
	kfree(rfkill_data);

err_data_alloc:
out:
	return ret;
}

static int rfkill_bluedroid_remove(struct platform_device *pdev)
{
	struct rfkill_bluedroid_data *rfkill_data = platform_get_drvdata(pdev);
	struct rfkill *rf_kill = rfkill_data->rf_kill;

	rfkill_unregister(rf_kill);
	rfkill_destroy(rf_kill);

	kfree(rfkill_data);

	return 0;
}

static const struct of_device_id  rfkill_bluedroid_ids[] = {
	{ .compatible = "fsl,rfkill-bluedroid", },
	{ /* sentinel */ }
};

static struct platform_driver rfkill_bluedroid_driver = {
	.probe = rfkill_bluedroid_probe,
	.remove = rfkill_bluedroid_remove,
	.driver = {
		.name = "rfkill-bluedroid",
		.of_match_table = of_match_ptr(rfkill_bluedroid_ids),
	},
};

module_platform_driver(rfkill_bluedroid_driver);

MODULE_AUTHOR("Guiming Zhuo <gmzhuo@gmail.com>");
MODULE_AUTHOR("Antonio Ospite <ospite@studenti.unina.it>");
MODULE_DESCRIPTION("Regulator consumer driver for rfkill");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rfkill-bluedroid");
