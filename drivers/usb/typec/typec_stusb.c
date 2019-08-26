// SPDX-License-Identifier: GPL-2.0
/*
 * STMicroelectronics STUSB Type-C controller family driver
 *
 * Copyright (C) 2019, STMicroelectronics
 * Author(s): Amelie Delaunay <amelie.delaunay@st.com>
 */

#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/usb/typec.h>

#define STUSB_ALERT_STATUS			0x0B /* RC */
#define STUSB_ALERT_STATUS_MASK_CTRL		0x0C /* RW */
#define STUSB_CC_CONNECTION_STATUS_TRANS	0x0D /* RC */
#define STUSB_CC_CONNECTION_STATUS		0x0E /* RO */
#define STUSB_MONITORING_STATUS_TRANS		0x0F /* RC */
#define STUSB_MONITORING_STATUS			0x10 /* RO */
#define STUSB_CC_OPERATION_STATUS		0x11 /* RO */
#define STUSB_HW_FAULT_STATUS_TRANS		0x12 /* RC */
#define STUSB_HW_FAULT_STATUS			0x13 /* RO */
#define STUSB_CC_CAPABILITY_CTRL		0x18 /* RW */
#define STUSB_CC_VCONN_SWITCH_CTRL		0x1E /* RW */
#define STUSB_VCONN_MONITORING_CTRL		0x20 /* RW */
#define STUSB_VBUS_MONITORING_RANGE_CTRL	0x22 /* RW */
#define STUSB_RESET_CTRL			0x23 /* RW */
#define STUSB_VBUS_DISCHARGE_TIME_CTRL		0x25 /* RW */
#define STUSB_VBUS_DISCHARGE_STATUS		0x26 /* RO */
#define STUSB_VBUS_ENABLE_STATUS		0x27 /* RO */
#define STUSB_CC_POWER_MODE_CTRL		0x28 /* RW */
#define STUSB_VBUS_MONITORING_CTRL		0x2E /* RW */
#define STUSB1600_REG_MAX			0x2F /* RO - Reserved */

/* STUSB_ALERT_STATUS/STUSB_ALERT_STATUS_MASK_CTRL bitfields */
#define STUSB_HW_FAULT				BIT(4)
#define STUSB_MONITORING			BIT(5)
#define STUSB_CC_CONNECTION			BIT(6)
#define STUSB_ALL_ALERTS			GENMASK(6, 4)

/* STUSB_CC_CONNECTION_STATUS_TRANS bitfields */
#define STUSB_CC_ATTACH_TRANS			BIT(0)

/* STUSB_CC_CONNECTION_STATUS bitfields */
#define STUSB_CC_ATTACH				BIT(0)
#define STUSB_CC_VCONN_SUPPLY			BIT(1)
#define STUSB_CC_DATA_ROLE(s)			(!!((s) & BIT(2)))
#define STUSB_CC_POWER_ROLE(s)			(!!((s) & BIT(3)))
#define STUSB_CC_ATTACHED_MODE			GENMASK(7, 5)

/* STUSB_MONITORING_STATUS_TRANS bitfields */
#define STUSB_VCONN_PRESENCE_TRANS		BIT(0)
#define STUSB_VBUS_PRESENCE_TRANS		BIT(1)
#define STUSB_VBUS_VSAFE0V_TRANS		BIT(2)
#define STUSB_VBUS_VALID_TRANS			BIT(3)

/* STUSB_MONITORING_STATUS bitfields */
#define STUSB_VCONN_PRESENCE			BIT(0)
#define STUSB_VBUS_PRESENCE			BIT(1)
#define STUSB_VBUS_VSAFE0V			BIT(2)
#define STUSB_VBUS_VALID			BIT(3)

/* STUSB_CC_OPERATION_STATUS bitfields */
#define STUSB_TYPEC_FSM_STATE			GENMASK(4, 0)
#define STUSB_SINK_POWER_STATE			GENMASK(6, 5)
#define STUSB_CC_ATTACHED			BIT(7)

/* STUSB_HW_FAULT_STATUS_TRANS bitfields */
#define STUSB_VCONN_SW_OVP_FAULT_TRANS		BIT(0)
#define STUSB_VCONN_SW_OCP_FAULT_TRANS		BIT(1)
#define STUSB_VCONN_SW_RVP_FAULT_TRANS		BIT(2)
#define STUSB_VPU_VALID_TRANS			BIT(4)
#define STUSB_VPU_OVP_FAULT_TRANS		BIT(5)
#define STUSB_THERMAL_FAULT			BIT(7)

/* STUSB_HW_FAULT_STATUS bitfields */
#define STUSB_VCONN_SW_OVP_FAULT_CC2		BIT(0)
#define STUSB_VCONN_SW_OVP_FAULT_CC1		BIT(1)
#define STUSB_VCONN_SW_OCP_FAULT_CC2		BIT(2)
#define STUSB_VCONN_SW_OCP_FAULT_CC1		BIT(3)
#define STUSB_VCONN_SW_RVP_FAULT_CC2		BIT(4)
#define STUSB_VCONN_SW_RVP_FAULT_CC1		BIT(5)
#define STUSB_VPU_VALID				BIT(6)
#define STUSB_VPU_OVP_FAULT			BIT(7)

/* STUSB_CC_CAPABILITY_CTRL bitfields */
#define STUSB_CC_VCONN_SUPPLY_EN		BIT(0)
#define STUSB_CC_VCONN_DISCHARGE_EN		BIT(4)
#define STUSB_CC_CURRENT_ADVERTISED		GENMASK(7, 6)

/* STUSB_VCONN_SWITCH_CTRL bitfields */
#define STUSB_CC_VCONN_SWITCH_ILIM		GENMASK(3, 0)

/* STUSB_VCONN_MONITORING_CTRL bitfields */
#define STUSB_VCONN_UVLO_THRESHOLD		BIT(6)
#define STUSB_VCONN_MONITORING_EN		BIT(7)

/* STUSB_VBUS_MONITORING_RANGE_CTRL bitfields */
#define STUSB_SHIFT_LOW_VBUS_LIMIT		GENMASK(3, 0)
#define STUSB_SHIFT_HIGH_VBUS_LIMIT		GENMASK(7, 4)

/* STUSB_RESET_CTRL bitfields */
#define STUSB_SW_RESET_EN			BIT(0)

/* STUSB_VBUS_DISCHARGE_TIME_CTRL bitfields */
#define STUSBXX02_VBUS_DISCHARGE_TIME_TO_PDO	GENMASK(3, 0)
#define STUSB_VBUS_DISCHARGE_TIME_TO_0V		GENMASK(7, 4)

/* STUSB_VBUS_DISCHARGE_STATUS bitfields */
#define STUSB_VBUS_DISCHARGE_EN			BIT(7)

/* STUSB_VBUS_ENABLE_STATUS bitfields */
#define STUSB_VBUS_SOURCE_EN			BIT(0)
#define STUSB_VBUS_SINK_EN			BIT(1)

/* STUSB_CC_POWER_MODE_CTRL bitfields */
#define STUSB_CC_POWER_MODE			GENMASK(2, 0)

/* STUSB_VBUS_MONITORING_CTRL bitfields */
#define STUSB_VDD_UVLO_DISABLE			BIT(0)
#define STUSB_VBUS_VSAFE0V_THRESHOLD		GENMASK(2, 1)
#define STUSB_VBUS_RANGE_DISABLE		BIT(4)
#define STUSB_VDD_OVLO_DISABLE			BIT(6)

enum stusb_pwr_mode {
	SOURCE_WITH_ACCESSORY,
	SINK_WITH_ACCESSORY,
	SINK_WITHOUT_ACCESSORY,
	DUAL_WITH_ACCESSORY,
	DUAL_WITH_ACCESSORY_AND_TRY_SRC,
	DUAL_WITH_ACCESSORY_AND_TRY_SNK,
};

struct stusb {
	struct device		*dev;
	struct regmap		*regmap;
	struct regulator	*vdd_supply;
	struct regulator	*vsys_supply;
	struct regulator	*vconn_supply;

	struct typec_port	*port;
	struct typec_capability capability;

	enum typec_port_type	port_type;
	enum typec_pwr_opmode	pwr_opmode;
};

static bool stusb_reg_writeable(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case STUSB_ALERT_STATUS_MASK_CTRL:
	case STUSB_CC_CAPABILITY_CTRL:
	case STUSB_CC_VCONN_SWITCH_CTRL:
	case STUSB_VCONN_MONITORING_CTRL:
	case STUSB_VBUS_MONITORING_RANGE_CTRL:
	case STUSB_RESET_CTRL:
	case STUSB_VBUS_DISCHARGE_TIME_CTRL:
	case STUSB_CC_POWER_MODE_CTRL:
	case STUSB_VBUS_MONITORING_CTRL:
		return true;
	default:
		return false;
	}
}

static bool stusb_reg_readable(struct device *dev, unsigned int reg)
{
	if ((reg >= 0x00 && reg <= 0x0A) ||
	    (reg >= 0x14 && reg <= 0x17) ||
	    (reg >= 0x19 && reg <= 0x1D) ||
	    (reg >= 0x29 && reg <= 0x2D) ||
	    (reg == 0x1F || reg == 0x21 || reg == 0x24 || reg == 0x2F))
		return false;
	else
		return true;
}

static bool stusb_reg_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case STUSB_ALERT_STATUS:
	case STUSB_CC_CONNECTION_STATUS_TRANS:
	case STUSB_CC_CONNECTION_STATUS:
	case STUSB_MONITORING_STATUS_TRANS:
	case STUSB_MONITORING_STATUS:
	case STUSB_CC_OPERATION_STATUS:
	case STUSB_HW_FAULT_STATUS_TRANS:
	case STUSB_HW_FAULT_STATUS:
	case STUSB_VBUS_DISCHARGE_STATUS:
	case STUSB_VBUS_ENABLE_STATUS:
		return true;
	default:
		return false;
	}
}

static bool stusb_reg_precious(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case STUSB_ALERT_STATUS:
	case STUSB_CC_CONNECTION_STATUS_TRANS:
	case STUSB_MONITORING_STATUS_TRANS:
	case STUSB_HW_FAULT_STATUS_TRANS:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config stusb1600_regmap_config = {
	.reg_bits	= 8,
	.reg_stride	= 1,
	.val_bits	= 8,
	.max_register	= STUSB1600_REG_MAX,
	.writeable_reg	= stusb_reg_writeable,
	.readable_reg	= stusb_reg_readable,
	.volatile_reg	= stusb_reg_volatile,
	.precious_reg	= stusb_reg_precious,
	.cache_type	= REGCACHE_RBTREE,
};

static bool stusb_get_vconn(struct stusb *chip)
{
	u32 val;
	int ret;

	ret = regmap_read(chip->regmap, STUSB_CC_CAPABILITY_CTRL, &val);
	if (ret) {
		dev_err(chip->dev, "Unable to get Vconn status: %d\n", ret);
		return false;
	}

	return !!FIELD_GET(STUSB_CC_VCONN_SUPPLY_EN, val);;
}

static int stusb_set_vconn(struct stusb *chip, bool on)
{
	int ret;

	/* Manage VCONN input supply */
	if (chip->vconn_supply) {
		if (on) {
			ret = regulator_enable(chip->vconn_supply);
			if (ret) {
				dev_err(chip->dev,
					"failed to enable vconn supply: %d\n",
					ret);
				return ret;
			}
		} else {
			regulator_disable(chip->vconn_supply);
		}
	}

	/* Manage VCONN monitoring and power path */
	ret = regmap_update_bits(chip->regmap, STUSB_VCONN_MONITORING_CTRL,
				 STUSB_VCONN_MONITORING_EN,
				 on ? STUSB_VCONN_MONITORING_EN : 0);
	if (ret)
		goto vconn_reg_disable;

	return 0;

vconn_reg_disable:
	if (chip->vconn_supply && on)
		regulator_disable(chip->vconn_supply);

	return ret;
}

static enum typec_pwr_opmode stusb_get_pwr_opmode(struct stusb *chip)
{
	u32 val;
	int ret;

	ret = regmap_read(chip->regmap, STUSB_CC_CAPABILITY_CTRL, &val);
	if (ret) {
		dev_err(chip->dev, "Unable to get pwr opmode: %d\n", ret);
		return TYPEC_PWR_MODE_USB;
	}

	return FIELD_GET(STUSB_CC_CURRENT_ADVERTISED, val);
}

static int stusb_init(struct stusb *chip)
{
	int ret;

	/* Change the default Type-C power mode */
	if (chip->port_type == TYPEC_PORT_SRC)
		ret = regmap_update_bits(chip->regmap,
					 STUSB_CC_POWER_MODE_CTRL,
					 STUSB_CC_POWER_MODE,
					 SOURCE_WITH_ACCESSORY);
	else if (chip->port_type == TYPEC_PORT_SNK)
		ret = regmap_update_bits(chip->regmap,
					 STUSB_CC_POWER_MODE_CTRL,
					 STUSB_CC_POWER_MODE,
					 SINK_WITH_ACCESSORY);
	else /* (capability->type == TYPEC_PORT_DRP) */
		ret = regmap_update_bits(chip->regmap,
					 STUSB_CC_POWER_MODE_CTRL,
					 STUSB_CC_POWER_MODE,
					 DUAL_WITH_ACCESSORY);
	if (ret)
		return ret;

	if (chip->port_type == TYPEC_PORT_SNK)
		return 0;

	/* Change the default Type-C Source power operation mode capability */
	ret = regmap_update_bits(chip->regmap, STUSB_CC_CAPABILITY_CTRL,
				 STUSB_CC_CURRENT_ADVERTISED,
				 FIELD_PREP(STUSB_CC_CURRENT_ADVERTISED,
					    chip->pwr_opmode));
	if (ret)
		return ret;

	/* Manage Type-C Source Vconn supply */
	if (stusb_get_vconn(chip)) {
		ret = stusb_set_vconn(chip, true);
		if (ret)
			return ret;
	}

	/* Mask all events interrupts - to be unmasked with interrupt support */
	ret = regmap_update_bits(chip->regmap, STUSB_ALERT_STATUS_MASK_CTRL,
				 STUSB_ALL_ALERTS, STUSB_ALL_ALERTS);

	return ret;
}

static int stusb_fw_get_caps(struct stusb *chip)
{
	struct fwnode_handle *fwnode = device_get_named_child_node(chip->dev,
								   "connector");
	const char *cap_str;
	int ret;

	if (!fwnode)
		return -EINVAL;

	chip->capability.fwnode = fwnode;

	ret = fwnode_property_read_string(fwnode, "power-role", &cap_str);
	if (!ret) {
		chip->port_type = typec_find_port_power_role(cap_str);
		if (chip->port_type < 0)
			return -EINVAL;

		chip->capability.type = chip->port_type;
	}

	if (chip->port_type == TYPEC_PORT_SNK)
		goto sink;

	if (chip->port_type == TYPEC_PORT_DRP)
		chip->capability.prefer_role = TYPEC_SINK;

	ret = fwnode_property_read_string(fwnode, "power-opmode", &cap_str);
	if (!ret) {
		chip->pwr_opmode = typec_find_port_power_opmode(cap_str);

		/* Power delivery not yet supported */
		if (chip->pwr_opmode < 0 ||
		    chip->pwr_opmode == TYPEC_PWR_MODE_PD) {
			dev_err(chip->dev, "bad power operation mode: %d\n",
				chip->pwr_opmode);
			return -EINVAL;
		}

	} else {
		chip->pwr_opmode = stusb_get_pwr_opmode(chip);
	}

sink:
	return 0;
}

static int stusb_get_caps(struct stusb *chip, bool *try)
{
	enum typec_port_type *type = &chip->capability.type;
	enum typec_port_data *data = &chip->capability.data;
	enum typec_accessory *accessory = chip->capability.accessory;
	u32 val;
	int ret;

	chip->capability.revision = USB_TYPEC_REV_1_2;

	ret = regmap_read(chip->regmap, STUSB_CC_POWER_MODE_CTRL, &val);
	if (ret)
		return ret;

	*try = false;

	switch (FIELD_GET(STUSB_CC_POWER_MODE, val)) {
	case SOURCE_WITH_ACCESSORY:
		*type = TYPEC_PORT_SRC;
		*data = TYPEC_PORT_DFP;
		*accessory++ = TYPEC_ACCESSORY_AUDIO;
		*accessory++ = TYPEC_ACCESSORY_DEBUG;
		break;
	case SINK_WITH_ACCESSORY:
		*type = TYPEC_PORT_SNK;
		*data = TYPEC_PORT_UFP;
		*accessory++ = TYPEC_ACCESSORY_AUDIO;
		*accessory++ = TYPEC_ACCESSORY_DEBUG;
		break;
	case SINK_WITHOUT_ACCESSORY:
		*type = TYPEC_PORT_SNK;
		*data = TYPEC_PORT_UFP;
		break;
	case DUAL_WITH_ACCESSORY:
		*type = TYPEC_PORT_DRP;
		*data = TYPEC_PORT_DRD;
		*accessory++ = TYPEC_ACCESSORY_AUDIO;
		*accessory++ = TYPEC_ACCESSORY_DEBUG;
		break;
	case DUAL_WITH_ACCESSORY_AND_TRY_SRC:
	case DUAL_WITH_ACCESSORY_AND_TRY_SNK:
		*type = TYPEC_PORT_DRP;
		*data = TYPEC_PORT_DRD;
		*accessory++ = TYPEC_ACCESSORY_AUDIO;
		*accessory++ = TYPEC_ACCESSORY_DEBUG;
		*try = true;
		break;
	default:
		return -EINVAL;
	}

	chip->port_type = *type;

	return stusb_fw_get_caps(chip);
}

static const struct of_device_id stusb_of_match[] = {
	{ .compatible = "st,stusb1600", .data = &stusb1600_regmap_config},
	{},
};

static int stusb_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct stusb *chip;
	const struct of_device_id *match;
	struct regmap_config *regmap_config;
	bool try_role;
	int ret;

	chip = devm_kzalloc(&client->dev, sizeof(struct stusb), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	i2c_set_clientdata(client, chip);

	match = i2c_of_match_device(stusb_of_match, client);
	regmap_config = (struct regmap_config *)match->data;
	chip->regmap = devm_regmap_init_i2c(client, regmap_config);
	if (IS_ERR(chip->regmap)) {
		ret = PTR_ERR(chip->regmap);
		dev_err(&client->dev,
			"Failed to allocate register map:%d\n", ret);
		return ret;
	}

	chip->dev = &client->dev;

	chip->vdd_supply = devm_regulator_get_optional(chip->dev, "vdd");
	if (IS_ERR(chip->vdd_supply)) {
		ret = PTR_ERR(chip->vdd_supply);
		if (ret != -ENODEV)
			return ret;
		chip->vdd_supply = NULL;
	} else {
		ret = regulator_enable(chip->vdd_supply);
		if (ret) {
			dev_err(chip->dev,
				"Failed to enable vdd supply: %d\n", ret);
			return ret;
		}
	}

	chip->vsys_supply = devm_regulator_get_optional(chip->dev, "vsys");
	if (IS_ERR(chip->vsys_supply)) {
		ret = PTR_ERR(chip->vsys_supply);
		if (ret != -ENODEV)
			goto vdd_reg_disable;
		chip->vsys_supply = NULL;
	} else {
		ret = regulator_enable(chip->vsys_supply);
		if (ret) {
			dev_err(chip->dev,
				"Failed to enable vsys supply: %d\n", ret);
			goto vdd_reg_disable;
		}
	}

	chip->vconn_supply = devm_regulator_get_optional(chip->dev, "vconn");
	if (IS_ERR(chip->vconn_supply)) {
		ret = PTR_ERR(chip->vconn_supply);
		if (ret != -ENODEV)
			goto vsys_reg_disable;
		chip->vconn_supply = NULL;
	}

	ret = stusb_get_caps(chip, &try_role);
	if (ret) {
		dev_err(chip->dev, "failed to get port caps: %d\n", ret);
		goto vsys_reg_disable;
	}

	ret = stusb_init(chip);
	if (ret) {
		dev_err(chip->dev, "failed to init port: %d\n", ret);
		goto vsys_reg_disable;
	}

	chip->port = typec_register_port(chip->dev, &chip->capability);
	if (!chip->port) {
		ret = -ENODEV;
		goto all_reg_disable;
	}

	/* To be moved in attach/detach procedure with interrupt support */
	typec_set_pwr_opmode(chip->port, chip->pwr_opmode);

	dev_info(chip->dev, "STUSB driver registered\n");

	return 0;

all_reg_disable:
	if (stusb_get_vconn(chip))
		stusb_set_vconn(chip, false);
vsys_reg_disable:
	if (chip->vsys_supply)
		regulator_disable(chip->vsys_supply);
vdd_reg_disable:
	if (chip->vdd_supply)
		regulator_disable(chip->vdd_supply);

	return ret;
}

static int stusb_remove(struct i2c_client *client)
{
	struct stusb *chip = i2c_get_clientdata(client);

	typec_unregister_port(chip->port);

  	if (stusb_get_vconn(chip))
		stusb_set_vconn(chip, false);

	if (chip->vdd_supply)
		regulator_disable(chip->vdd_supply);

	if (chip->vsys_supply)
		regulator_disable(chip->vsys_supply);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int stusb_resume(struct device *dev)
{
	struct stusb *chip = dev_get_drvdata(dev);

	return stusb_init(chip);
}
#endif

static SIMPLE_DEV_PM_OPS(stusb_pm_ops, NULL, stusb_resume);

static struct i2c_driver stusb_driver = {
	.driver = {
		.name = "typec_stusb",
		.pm = &stusb_pm_ops,
		.of_match_table = stusb_of_match,
	},
	.probe = stusb_probe,
	.remove = stusb_remove,
};
module_i2c_driver(stusb_driver);

MODULE_AUTHOR("Amelie Delaunay <amelie.delaunay@st.com>");
MODULE_DESCRIPTION("STMicroelectronics STUSB Type-C controller driver");
MODULE_LICENSE("GPL v2");
