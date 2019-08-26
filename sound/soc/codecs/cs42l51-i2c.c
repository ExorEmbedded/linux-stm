/*
 * cs42l56.c -- CS42L51 ALSA SoC I2C audio driver
 *
 * Copyright 2014 CirrusLogic, Inc.
 *
 * Author: Brian Austin <brian.austin@cirrus.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <sound/soc.h>

#include "cs42l51.h"

static struct i2c_device_id cs42l51_i2c_id[] = {
	{"cs42l51", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, cs42l51_i2c_id);

static int cs42l51_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct regmap_config config;

	config = cs42l51_regmap;

	return cs42l51_probe(&i2c->dev, devm_regmap_init_i2c(i2c, &config));
}

static int cs42l51_i2c_remove(struct i2c_client *i2c)
{
	return cs42l51_remove(&i2c->dev);
}

#ifdef CONFIG_PM_SLEEP
static int cs42l51_suspend(struct device *dev)
{
	struct cs42l51_private *cs42l51 = dev_get_drvdata(dev);

	regcache_cache_only(cs42l51->regmap, true);
	regcache_mark_dirty(cs42l51->regmap);

	return 0;
}

static int cs42l51_resume(struct device *dev)
{
	struct cs42l51_private *cs42l51 = dev_get_drvdata(dev);

	regcache_cache_only(cs42l51->regmap, false);

	return regcache_sync(cs42l51->regmap);
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops cs42l51_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(cs42l51_suspend, cs42l51_resume)
};

static struct i2c_driver cs42l51_i2c_driver = {
	.driver = {
		.name = "cs42l51",
		.of_match_table = cs42l51_of_match,
		.pm = &cs42l51_pm_ops,
	},
	.probe = cs42l51_i2c_probe,
	.remove = cs42l51_i2c_remove,
	.id_table = cs42l51_i2c_id,
};

module_i2c_driver(cs42l51_i2c_driver);

MODULE_DESCRIPTION("ASoC CS42L51 I2C Driver");
MODULE_AUTHOR("Brian Austin, Cirrus Logic Inc, <brian.austin@cirrus.com>");
MODULE_LICENSE("GPL");
