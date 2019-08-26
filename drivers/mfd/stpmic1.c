// SPDX-License-Identifier: GPL-2.0
// Copyright (C) STMicroelectronics 2018
// Author: Pascal Paillet <p.paillet@st.com>

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/mfd/stpmic1.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/pm_wakeirq.h>
#include <linux/regmap.h>

#include <dt-bindings/mfd/st,stpmic1.h>

#define STPMIC1_MAIN_IRQ 0
#define STPMIC1_WAKEUP_IRQ 1

static bool stpmic1_reg_readable(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TURN_ON_SR:
	case TURN_OFF_SR:
	case ICC_LDO_TURN_OFF_SR:
	case ICC_BUCK_TURN_OFF_SR:
	case RREQ_STATE_SR:
	case VERSION_SR:
	case SWOFF_PWRCTRL_CR:
	case PADS_PULL_CR:
	case BUCKS_PD_CR:
	case LDO14_PD_CR:
	case LDO56_VREF_PD_CR:
	case VBUS_DET_VIN_CR:
	case PKEY_TURNOFF_CR:
	case BUCKS_MASK_RANK_CR:
	case BUCKS_MASK_RESET_CR:
	case LDOS_MASK_RANK_CR:
	case LDOS_MASK_RESET_CR:
	case WCHDG_CR:
	case WCHDG_TIMER_CR:
	case BUCKS_ICCTO_CR:
	case LDOS_ICCTO_CR:
	case BUCK1_ACTIVE_CR:
	case BUCK2_ACTIVE_CR:
	case BUCK3_ACTIVE_CR:
	case BUCK4_ACTIVE_CR:
	case VREF_DDR_ACTIVE_CR:
	case LDO1_ACTIVE_CR:
	case LDO2_ACTIVE_CR:
	case LDO3_ACTIVE_CR:
	case LDO4_ACTIVE_CR:
	case LDO5_ACTIVE_CR:
	case LDO6_ACTIVE_CR:
	case BUCK1_STDBY_CR:
	case BUCK2_STDBY_CR:
	case BUCK3_STDBY_CR:
	case BUCK4_STDBY_CR:
	case VREF_DDR_STDBY_CR:
	case LDO1_STDBY_CR:
	case LDO2_STDBY_CR:
	case LDO3_STDBY_CR:
	case LDO4_STDBY_CR:
	case LDO5_STDBY_CR:
	case LDO6_STDBY_CR:
	case BST_SW_CR:
	case INT_PENDING_R1:
	case INT_PENDING_R2:
	case INT_PENDING_R3:
	case INT_PENDING_R4:
	case INT_DBG_LATCH_R1:
	case INT_DBG_LATCH_R2:
	case INT_DBG_LATCH_R3:
	case INT_DBG_LATCH_R4:
	case INT_CLEAR_R1:
	case INT_CLEAR_R2:
	case INT_CLEAR_R3:
	case INT_CLEAR_R4:
	case INT_MASK_R1:
	case INT_MASK_R2:
	case INT_MASK_R3:
	case INT_MASK_R4:
	case INT_SET_MASK_R1:
	case INT_SET_MASK_R2:
	case INT_SET_MASK_R3:
	case INT_SET_MASK_R4:
	case INT_CLEAR_MASK_R1:
	case INT_CLEAR_MASK_R2:
	case INT_CLEAR_MASK_R3:
	case INT_CLEAR_MASK_R4:
	case INT_SRC_R1:
	case INT_SRC_R2:
	case INT_SRC_R3:
	case INT_SRC_R4:
		return true;
	default:
		return false;
	}
}

static bool stpmic1_reg_writeable(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SWOFF_PWRCTRL_CR:
	case PADS_PULL_CR:
	case BUCKS_PD_CR:
	case LDO14_PD_CR:
	case LDO56_VREF_PD_CR:
	case VBUS_DET_VIN_CR:
	case PKEY_TURNOFF_CR:
	case BUCKS_MASK_RANK_CR:
	case BUCKS_MASK_RESET_CR:
	case LDOS_MASK_RANK_CR:
	case LDOS_MASK_RESET_CR:
	case WCHDG_CR:
	case WCHDG_TIMER_CR:
	case BUCKS_ICCTO_CR:
	case LDOS_ICCTO_CR:
	case BUCK1_ACTIVE_CR:
	case BUCK2_ACTIVE_CR:
	case BUCK3_ACTIVE_CR:
	case BUCK4_ACTIVE_CR:
	case VREF_DDR_ACTIVE_CR:
	case LDO1_ACTIVE_CR:
	case LDO2_ACTIVE_CR:
	case LDO3_ACTIVE_CR:
	case LDO4_ACTIVE_CR:
	case LDO5_ACTIVE_CR:
	case LDO6_ACTIVE_CR:
	case BUCK1_STDBY_CR:
	case BUCK2_STDBY_CR:
	case BUCK3_STDBY_CR:
	case BUCK4_STDBY_CR:
	case VREF_DDR_STDBY_CR:
	case LDO1_STDBY_CR:
	case LDO2_STDBY_CR:
	case LDO3_STDBY_CR:
	case LDO4_STDBY_CR:
	case LDO5_STDBY_CR:
	case LDO6_STDBY_CR:
	case BST_SW_CR:
	case INT_DBG_LATCH_R1:
	case INT_DBG_LATCH_R2:
	case INT_DBG_LATCH_R3:
	case INT_DBG_LATCH_R4:
	case INT_CLEAR_R1:
	case INT_CLEAR_R2:
	case INT_CLEAR_R3:
	case INT_CLEAR_R4:
	case INT_SET_MASK_R1:
	case INT_SET_MASK_R2:
	case INT_SET_MASK_R3:
	case INT_SET_MASK_R4:
	case INT_CLEAR_MASK_R1:
	case INT_CLEAR_MASK_R2:
	case INT_CLEAR_MASK_R3:
	case INT_CLEAR_MASK_R4:
		return true;
	default:
		return false;
	}
}

static bool stpmic1_reg_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TURN_ON_SR:
	case TURN_OFF_SR:
	case ICC_LDO_TURN_OFF_SR:
	case ICC_BUCK_TURN_OFF_SR:
	case RREQ_STATE_SR:
	case INT_PENDING_R1:
	case INT_PENDING_R2:
	case INT_PENDING_R3:
	case INT_PENDING_R4:
	case INT_SRC_R1:
	case INT_SRC_R2:
	case INT_SRC_R3:
	case INT_SRC_R4:
	case WCHDG_CR:
		return true;
	default:
		return false;
	}
}

const struct regmap_config stpmic1_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.max_register = PMIC_MAX_REGISTER_ADDRESS,
	.readable_reg = stpmic1_reg_readable,
	.writeable_reg = stpmic1_reg_writeable,
	.volatile_reg = stpmic1_reg_volatile,
};

static const struct regmap_irq stpmic1_irqs[] = {
	[IT_PONKEY_F]		= { .mask = 0x01 },
	[IT_PONKEY_R]		= { .mask = 0x02 },
	[IT_WAKEUP_F]		= { .mask = 0x04 },
	[IT_WAKEUP_R]		= { .mask = 0x08 },
	[IT_VBUS_OTG_F]		= { .mask = 0x10 },
	[IT_VBUS_OTG_R]		= { .mask = 0x20 },
	[IT_SWOUT_F]		= { .mask = 0x40 },
	[IT_SWOUT_R]		= { .mask = 0x80 },

	[IT_CURLIM_BUCK1]	= { .reg_offset = 1, .mask = 0x01 },
	[IT_CURLIM_BUCK2]	= { .reg_offset = 1, .mask = 0x02 },
	[IT_CURLIM_BUCK3]	= { .reg_offset = 1, .mask = 0x04 },
	[IT_CURLIM_BUCK4]	= { .reg_offset = 1, .mask = 0x08 },
	[IT_OCP_OTG]		= { .reg_offset = 1, .mask = 0x10 },
	[IT_OCP_SWOUT]		= { .reg_offset = 1, .mask = 0x20 },
	[IT_OCP_BOOST]		= { .reg_offset = 1, .mask = 0x40 },
	[IT_OVP_BOOST]		= { .reg_offset = 1, .mask = 0x80 },

	[IT_CURLIM_LDO1]	= { .reg_offset = 2, .mask = 0x01 },
	[IT_CURLIM_LDO2]	= { .reg_offset = 2, .mask = 0x02 },
	[IT_CURLIM_LDO3]	= { .reg_offset = 2, .mask = 0x04 },
	[IT_CURLIM_LDO4]	= { .reg_offset = 2, .mask = 0x08 },
	[IT_CURLIM_LDO5]	= { .reg_offset = 2, .mask = 0x10 },
	[IT_CURLIM_LDO6]	= { .reg_offset = 2, .mask = 0x20 },
	[IT_SHORT_SWOTG]	= { .reg_offset = 2, .mask = 0x40 },
	[IT_SHORT_SWOUT]	= { .reg_offset = 2, .mask = 0x80 },

	[IT_TWARN_F]		= { .reg_offset = 3, .mask = 0x01 },
	[IT_TWARN_R]		= { .reg_offset = 3, .mask = 0x02 },
	[IT_VINLOW_F]		= { .reg_offset = 3, .mask = 0x04 },
	[IT_VINLOW_R]		= { .reg_offset = 3, .mask = 0x08 },
	[IT_SWIN_F]		= { .reg_offset = 3, .mask = 0x40 },
	[IT_SWIN_R]		= { .reg_offset = 3, .mask = 0x80 },
};

static const struct regmap_irq_chip stpmic1_regmap_irq_chip = {
	.name = "pmic_irq",
	.status_base = INT_PENDING_R1,
	.mask_base = INT_CLEAR_MASK_R1,
	.unmask_base = INT_SET_MASK_R1,
	.ack_base = INT_CLEAR_R1,
	.num_regs = STPMIC1_PMIC_NUM_IRQ_REGS,
	.irqs = stpmic1_irqs,
	.num_irqs = ARRAY_SIZE(stpmic1_irqs),
};

static int stpmic1_probe(struct i2c_client *i2c,
			 const struct i2c_device_id *id)
{
	struct stpmic1 *ddata;
	struct device *dev = &i2c->dev;
	int ret;
	struct device_node *np = dev->of_node;
	u32 reg;

	ddata = devm_kzalloc(dev, sizeof(struct stpmic1), GFP_KERNEL);
	if (!ddata)
		return -ENOMEM;

	i2c_set_clientdata(i2c, ddata);
	ddata->dev = dev;

	ddata->regmap = devm_regmap_init_i2c(i2c, &stpmic1_regmap_config);
	if (IS_ERR(ddata->regmap))
		return PTR_ERR(ddata->regmap);

	ddata->irq = of_irq_get(np, STPMIC1_MAIN_IRQ);
	if (ddata->irq < 0) {
		dev_err(dev, "Failed to get main IRQ: %d\n", ddata->irq);
		return ddata->irq;
	}

	if (of_property_read_bool(np, "wakeup-source"))
		device_init_wakeup(dev, true);

	if (!of_property_read_u32(np, "st,main-control-register", &reg)) {
		ret = regmap_update_bits(ddata->regmap,
					 SWOFF_PWRCTRL_CR,
					 PWRCTRL_POLARITY_HIGH |
					 PWRCTRL_PIN_VALID |
					 RESTART_REQUEST_ENABLED,
					 reg);
		if (ret) {
			dev_err(dev,
				"Failed to update main control register: %d\n",
				ret);
			return ret;
		}
	}

	/* Read Version ID */
	ret = regmap_read(ddata->regmap, VERSION_SR, &reg);
	if (ret) {
		dev_err(dev, "Unable to read pmic version\n");
		return ret;
	}
	dev_info(dev, "PMIC Chip Version: 0x%x\n", reg);

	if (!of_property_read_u32(np, "st,pads-pull-register", &reg)) {
		ret = regmap_update_bits(ddata->regmap,
					 PADS_PULL_CR,
					 WAKEUP_DETECTOR_DISABLED |
					 PWRCTRL_PD_ACTIVE |
					 PWRCTRL_PU_ACTIVE |
					 WAKEUP_PD_ACTIVE,
					 reg);
		if (ret) {
			dev_err(dev,
				"Failed to update pads control register: %d\n",
				ret);
			return ret;
		}
	}

	if (!of_property_read_u32(np, "st,vin-control-register", &reg)) {
		ret = regmap_update_bits(ddata->regmap,
					 VBUS_DET_VIN_CR,
					 VINLOW_CTRL_REG_MASK,
					 reg);
		if (ret) {
			dev_err(dev,
				"Failed to update vin control register: %d\n",
				ret);
			return ret;
		}
	}

	if (!of_property_read_u32(np, "st,usb-control-register", &reg)) {
		ret = regmap_update_bits(ddata->regmap, BST_SW_CR,
					 BOOST_OVP_DISABLED |
					 VBUS_OTG_DETECTION_DISABLED |
					 SW_OUT_DISCHARGE |
					 VBUS_OTG_DISCHARGE |
					 OCP_LIMIT_HIGH,
					 reg);
		if (ret) {
			dev_err(dev,
				"Failed to update usb control register: %d\n",
				ret);
			return ret;
		}
	}

	/* Initialize PMIC IRQ Chip & IRQ domains associated */
	ret = devm_regmap_add_irq_chip(dev, ddata->regmap, ddata->irq,
				       IRQF_ONESHOT | IRQF_SHARED,
				       0, &stpmic1_regmap_irq_chip,
				       &ddata->irq_data);
	if (ret) {
		dev_err(dev, "IRQ Chip registration failed: %d\n", ret);
		return ret;
	}

	return devm_of_platform_populate(dev);
}

static const struct i2c_device_id stpmic1_id[] = {
	{ "stpmic1"},
	{}
};

MODULE_DEVICE_TABLE(i2c, stpmic1_id);

#ifdef CONFIG_PM_SLEEP
static int stpmic1_suspend(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct stpmic1 *pmic_dev = i2c_get_clientdata(i2c);

	disable_irq(pmic_dev->irq);
	if (device_may_wakeup(dev))
		enable_irq_wake(pmic_dev->irq);

	return 0;
}

static int stpmic1_resume(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct stpmic1 *pmic_dev = i2c_get_clientdata(i2c);
	int ret;

	ret = regcache_sync(pmic_dev->regmap);
	if (ret)
		return ret;

	if (device_may_wakeup(dev))
		disable_irq_wake(pmic_dev->irq);

	enable_irq(pmic_dev->irq);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(stpmic1_pm, stpmic1_suspend, stpmic1_resume);

static struct i2c_driver stpmic1_driver = {
	.driver = {
		.name = "stpmic1",
		.pm = &stpmic1_pm,
	},
	.probe = stpmic1_probe,
	.id_table = stpmic1_id,
};

module_i2c_driver(stpmic1_driver);

MODULE_DESCRIPTION("STPMIC1 PMIC Driver");
MODULE_AUTHOR("Pascal Paillet <p.paillet@st.com>");
MODULE_LICENSE("GPL v2");
