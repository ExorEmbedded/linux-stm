/*
 *  gpio_initializer.c - Linux kernel module to allow gpio initial configuration via dtb.
 *                             
 *  The specified gpio flags will allow to define each gpio direction, polarity, inversion, initial level (if output), drive mode and visibility in sysfs.
 *  The specified gpio-names list will define the label to be assigned to each gpio in sysfs, if visible there.
 *
 *  Written by: Giovanni Pavoni, Exor S.p.a.
 *  Copyright (c) 2018 Exor S.p.a.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#ifdef CONFIG_OF
static int gpinit_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	int gpio, gpio_count, gpio_name_count, i;
	enum of_gpio_flags flags;
	const char *name;

	gpio_count = of_gpio_named_count(np, "gpios");
	printk("%s gpio_count=%d\n",__func__,gpio_count);

	if (gpio_count < 0) {
		dev_err(dev, "missing gpios: %d\n", gpio_count);
		return gpio_count;
	}

	gpio_name_count = of_property_count_strings(np, "gpio-names");
	if (gpio_count != gpio_name_count) {
		dev_err(dev, "number of gpios does not equal number of gpio names\n");
		return -EINVAL;
	}
	
	/* Loop for each specified gpio */
	for (i = 0; i < gpio_count; i++) 
	{
	  gpio = of_get_named_gpio_flags(np, "gpios", i, &flags);
	  if ((gpio < 0) || (!gpio_is_valid(gpio)))
	  {
	    dev_err(dev, "Could not get gpio %d\n", i);
	    return -EINVAL;
	  }
	  
	  if(of_property_read_string_index(np, "gpio-names", i, &name))
	  {
	    dev_err(dev, "Could not get gpio name %d\n", i);
	    return -EINVAL;
	  }
	  printk("i=%d gpio=%d flags=0x%x name=%s\n",i, gpio, flags, name);
	  
	  if(gpio_request_one(gpio, flags, name))
	  {
	    dev_err(dev, "Could not request gpio %d\n", i);
	    return -EINVAL;
	  }
	  
	  if(flags & GPIOF_EXPORT)
	    if(gpio_export_link(dev, name, gpio))
	    {
	      dev_err(dev, "Could not export gpio link %d\n", i);
	      return -EINVAL;
	    }
	}
	return 0;
}

#else
static int gpinit_parse_dt(struct device *dev)
{
    return -ENODEV;
}
#endif


/*
 * Probe and remove functions
 */
static int gpinit_probe(struct platform_device *pdev)
{
    int res = 0;

    res = gpinit_parse_dt(&pdev->dev);
    if (res) {
        dev_err(&pdev->dev, "Could not find valid DT data.\n");
        goto lbl_error1;
    }

    printk("%s : SUCCESS\n", __func__);
    return res;
lbl_error1:
     return res;
}

static int gpinit_remove(struct platform_device *pdev)
{
    return 0;
}

/*
 * Driver instantiation
 */
#ifdef CONFIG_OF
static struct of_device_id gpinit_of_match[] = {
{ .compatible = "exor,gpio_initializer" },
{ }
};

MODULE_DEVICE_TABLE(of, gpinit_of_match);
#endif

static struct platform_driver gpinit_driver = {
    .driver		= {
        .name		= "gpio_initializer",
        .owner		= THIS_MODULE,
        .of_match_table	= of_match_ptr(gpinit_of_match),
    },
    .probe		= gpinit_probe,
    .remove		= gpinit_remove,
};

module_platform_driver(gpinit_driver);

MODULE_DESCRIPTION("gpio_initializer");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:gpio_initializer");
