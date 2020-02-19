/*
 *  tja1145.c - Linux kernel module for TJA1145 transceiver
 *
 *  Written by: Luigi Scagnet, Exor S.p.a.
 *  Copyright (c) 2017 Exor S.p.a.
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
 *
 *  mount -t debugfs none /sys/kernel/debug
 *  ls -hal --color /sys/kernel/debug
 *  echo -n 'file tja1145.c +fp' > /sys/kernel/debug/dynamic_debug/control
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/can.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/if.h>
#include <linux/uaccess.h>
#include <linux/spi/spi.h>
#include <linux/spi/tja1145.h>

static DEFINE_MUTEX(spi_lock);

struct tja1145_data {
	struct tja1145_functions_accessor func_acc;
	struct spi_device	*spi;
};

static const struct spi_device_id tja1145_id[] = {
	{ "tja1145x", 0 },
	{ "tja1145xFD", 1 },
	{ }
};

/*
 * Only for DEBUG
 * regs_list
 */
static u8 regs_list[] = {
	REG_MODE_CONTROL,
	REG_MAIN_STATUS,
	REG_SYSTEM_EVENT_ENABLE,
	REG_MEMORY_0,
	REG_MEMORY_1,
	REG_MEMORY_2,
	REG_MEMORY_3,
	REG_LOCK_CONTROL,
	REG_CAN_CONTROL,
	REG_TRANSCEIVER_STATUS,
	REG_TRANSCEIVER_EVENT_ENABLE,
	REG_DATA_RATE,
	REG_IDENTIFIER_0,
	REG_IDENTIFIER_1,
	REG_IDENTIFIER_2,
	REG_IDENTIFIER_3,
	REG_MASK_0,
	REG_MASK_1,
	REG_MASK_2,
	REG_MASK_3,
	REG_FRAME_CONTROL,
	REG_DATA_MASK0,
	REG_DATA_MASK1,
	REG_DATA_MASK2,
	REG_DATA_MASK3,
	REG_DATA_MASK4,
	REG_DATA_MASK5,
	REG_DATA_MASK6,
	REG_DATA_MASK7,
	REG_WAKE_PIN_STATUS,
	REG_WAKE_PIN_ENABLE,
	REG_EVENT_CAPTURE_STATUS,
	REG_SYSTEM_EVENT_STATUS,
	REG_TRANSCEIVER_EVENT_STATUS,
	REG_WAKE_EVENT_STATUS,
	REG_IDENTIFICATION
};

/*
 * tja1145_driver_version --> guard function
 */
void tja1145_driver_version(struct spi_device*	spi)
{
	dev_info(&spi->dev, "Tja1145x driver version %d.%d \n",  MAJOR_DRIVER_VERS ,  MINOR_DRIVER_VERS );
}
EXPORT_SYMBOL(tja1145_driver_version);

/*
 * tja1145_read_single_reg
 */

static int tja1145_read_single_reg(struct spi_device *spi, u8 reg)
{
	u8  read_reg = ((reg << 1) | READ_ONLY);
	int ret;
	dev_dbg(&spi->dev, "%s Reg: %02X RegShift: %04X \n", __func__, reg, read_reg );

	mutex_lock(&spi_lock);
	ret = spi_w8r8(spi, read_reg);
	mutex_unlock(&spi_lock);

	dev_dbg(&spi->dev, "%s Reg: %02X Readed val: %d\n", __func__, reg, ret );

	return ret;
}

/*
 * tja1145_write_single_reg
 */
static int tja1145_write_single_reg(struct spi_device *spi, u8 reg, u8 data)
{
	int     status;
	u16  write_reg_data = ((data << 8) | (reg << 1));
	dev_dbg(&spi->dev, "%s Reg: %02X RegShift: %04X Data: %02X \n", __func__, reg, write_reg_data, data );

	mutex_lock(&spi_lock);
	status = spi_write(spi, &write_reg_data, 2 );
	mutex_unlock(&spi_lock);

	dev_dbg(&spi->dev, "%s Reg: %02X Data: %02X Write Status = %d\n", __func__, reg, data, status );
	return status;
}

/*
 * tja1145_get_chip_id
 */
static int tja1145_get_chip_id(struct spi_device *spi)
{
	int chip_id = tja1145_read_single_reg(spi, REG_IDENTIFICATION);

	switch(chip_id)
	{
	case WHOIAM_VALUE:
		dev_info(&spi->dev, "found TJA1145x\n" );
		break;

	case WHOIAM_VALUE_FD:
		dev_info(&spi->dev, "found TJA1145xFD\n" );
		break;
	default:
		dev_err(&spi->dev, "chip id %x not know\n", chip_id );
		return -ENODEV;
	}

	return chip_id;
}

/*
 * tja1145_dump_all_regs
 */
static void tja1145_dump_all_regs(struct spi_device *spi)
{
	u8 value, i;
	for(i=0; i<sizeof(regs_list)/regs_list[0]; i++)
	{
		value = tja1145_read_single_reg(spi, regs_list[i]);
		dev_dbg(&spi->dev, "%s Reg: %02X Value: %02X \n", __func__, regs_list[i], value );
	}
}

/*
 * tja1145_get_wake_event_status_and_clear
 */
static void tja1145_get_wake_event_status_and_clear(struct spi_device *spi)
{
	uint8_t tmp8u;
	dev_dbg(&spi->dev, "%s WAKE pin event pending\n", __func__);
	tmp8u = tja1145_read_single_reg(spi, REG_WAKE_EVENT_STATUS);
	if (tmp8u & (1<<1))
		dev_dbg(&spi->dev, "%s WPR - rising edge detected on WAKE pin\n", __func__);
	if (tmp8u & (1<<0))
		dev_dbg(&spi->dev, "%s WPF - falling edge detected on WAKE pin\n", __func__);
	tja1145_write_single_reg(spi, REG_WAKE_EVENT_STATUS, 0x03 );
}

/*
 * tja1145_get_transceiver_event_status_and_clear
 */
static void tja1145_get_transceiver_event_status_and_clear(struct spi_device *spi)
{
	uint8_t tmp8u;
	dev_dbg(&spi->dev, "%s Transceiver event pending\n", __func__);
	tmp8u = tja1145_read_single_reg(spi, REG_TRANSCEIVER_EVENT_STATUS);
	if (tmp8u & (1<<5))
		dev_dbg(&spi->dev, "%s PNFDE - partial networking frame detection error detected\n", __func__);
	if (tmp8u & (1<<4))
		dev_dbg(&spi->dev, "%s CBS - no activity on CAN bus\n", __func__);
	if (tmp8u & (1<<1))
		dev_dbg(&spi->dev, "%s CF - CAN failure event detected\n", __func__);
	if (tmp8u & (1<<0))
		dev_dbg(&spi->dev, "%s CW - CAN wake-up event detected\n", __func__);

	tja1145_write_single_reg(spi, REG_TRANSCEIVER_EVENT_STATUS, 0x33 );
}

/*
 * tja1145_get_system_event_status_and_clear
 */
static void tja1145_get_system_event_status_and_clear(struct spi_device *spi)
{
	uint8_t tmp8u;
	dev_dbg(&spi->dev, "%s System event pending\n", __func__);
	tmp8u = tja1145_read_single_reg(spi, REG_SYSTEM_EVENT_STATUS);
	if (tmp8u & (1<<4))
		dev_dbg(&spi->dev, "%s CPO - the TJA1145 has left Off mode after power-on\n", __func__);
	if (tmp8u & (1<<2))
		dev_dbg(&spi->dev, "%s OTW - Overtemperature detected\n", __func__);
	if (tmp8u & (1<<1))
		dev_dbg(&spi->dev, "%s SPIF - SPI failure detected\n", __func__);

	tja1145_write_single_reg(spi, REG_SYSTEM_EVENT_STATUS, 0x16 );

}

/*
 * tja1145_get_status_and_clear
 */
static void tja1145_get_status_and_clear(struct spi_device *spi)
{
	uint8_t tmp8u = tja1145_read_single_reg(spi, REG_EVENT_CAPTURE_STATUS);

	if( tmp8u & 0x0D )
	{
		if(tmp8u & (1<<3))
			tja1145_get_wake_event_status_and_clear(spi);
		if(tmp8u & (1<<2))
			tja1145_get_transceiver_event_status_and_clear(spi);
		if(tmp8u & (1<<0))
			tja1145_get_system_event_status_and_clear(spi);
	}

}

/*
 * tja1145_set_sleep_WUP_CAN
 */
static void tja1145_set_sleep_WUP_CAN(struct spi_device *spi)
{
	/*
	 *  Clear any pending errors
	 */
	tja1145_get_status_and_clear(spi);
	/*
	 *  Tell device not to wake up on local pin
	 */
	tja1145_write_single_reg(spi, REG_WAKE_PIN_ENABLE, 0x00 );// WPRE=0, WPFE=0
	/*
	 *  set wake on CAN (CWE = 1) and CAN selective wake-up (CPNC = 1)
	 */
	tja1145_write_single_reg(spi, REG_TRANSCEIVER_EVENT_ENABLE, 0x01 );// CWE = 1, CFE=0, CBSE=0
	/*
	 *  Enable  partial networking registers configured
	 *  Enable CAN selective wake-up
	 */
	tja1145_write_single_reg(spi, REG_CAN_CONTROL, (OFFLINE_MODE | (1<<5) | (1<<4)) );// CFDC=0, PNCOK=1, CPNC=1, CMC=Offline
	/*
	 *  Set transceiver to Sleep mode
	 */
	tja1145_write_single_reg(spi, REG_MODE_CONTROL, SLEEP_MODE );// WPRE=0, WPFE=0
}

/*
 * tja1145_configure_wake_can_disable
 */
static void tja1145_configure_wake_can_disable( struct spi_device *spi )
{
	dev_dbg(&spi->dev, "%s \n", __func__ );
	dev_info(&spi->dev, "Setup WakeUp over CAN Disable\n" );

	tja1145_write_single_reg(spi, REG_IDENTIFIER_0,				0x00 );
	tja1145_write_single_reg(spi, REG_IDENTIFIER_1,				0x00 );
	tja1145_write_single_reg(spi, REG_IDENTIFIER_2,				0x00 );
	tja1145_write_single_reg(spi, REG_IDENTIFIER_3,				0x00 );
	tja1145_write_single_reg(spi, REG_MASK_0,				0x00 );
	tja1145_write_single_reg(spi, REG_MASK_1,				0x00 );
	tja1145_write_single_reg(spi, REG_MASK_2,				0x00 );
	tja1145_write_single_reg(spi, REG_MASK_3,				0x00 );

	tja1145_write_single_reg(spi, REG_FRAME_CONTROL, EXTENDED_FRAME_FORMAT | DLC_DATA_WAKEUP | DLC_8 );
	tja1145_write_single_reg(spi, REG_DATA_MASK0,				0x00 );
	tja1145_write_single_reg(spi, REG_DATA_MASK1,				0x00 );
	tja1145_write_single_reg(spi, REG_DATA_MASK2,				0x00 );
	tja1145_write_single_reg(spi, REG_DATA_MASK3,				0x00 );
	tja1145_write_single_reg(spi, REG_DATA_MASK4,				0x00 );
	tja1145_write_single_reg(spi, REG_DATA_MASK5,				0x00 );
	tja1145_write_single_reg(spi, REG_DATA_MASK6,				0x00 );
	tja1145_write_single_reg(spi, REG_DATA_MASK7,				0x00 );

	tja1145_set_sleep_WUP_CAN(spi);
}

/*
 * tja1145_configure_wake_can_extendedId
 */
static void tja1145_configure_wake_can_extendedId( struct spi_device *spi, struct can_filter *wu_settings)
{
	dev_dbg(&spi->dev, "%s ---> Id: %x Mask: %x\n", __func__, wu_settings->can_id, wu_settings->can_mask );
	//Enable_TJA1145_CAN_WUP
	tja1145_write_single_reg(spi, REG_IDENTIFIER_0,	(wu_settings->can_id & 0x000000FF) >>  0 );
	tja1145_write_single_reg(spi, REG_IDENTIFIER_1,	(wu_settings->can_id & 0x0000FF00) >>  8 );
	tja1145_write_single_reg(spi, REG_IDENTIFIER_2,	(wu_settings->can_id & 0x00FF0000) >> 16 );
	tja1145_write_single_reg(spi, REG_IDENTIFIER_3,	(wu_settings->can_id & 0x1F000000) >> 24 );

	tja1145_write_single_reg(spi, REG_MASK_0,	(wu_settings->can_mask & 0x000000FF) >>  0 );
	tja1145_write_single_reg(spi, REG_MASK_1,	(wu_settings->can_mask & 0x0000FF00) >>  8 );
	tja1145_write_single_reg(spi, REG_MASK_2,	(wu_settings->can_mask & 0x00FF0000) >> 16 );
	tja1145_write_single_reg(spi, REG_MASK_3,	(wu_settings->can_mask & 0x1F000000) >> 24 );

	tja1145_write_single_reg(spi, REG_FRAME_CONTROL, EXTENDED_FRAME_FORMAT );
	//Set_TJA1145_Sleep_WUP_CAN
	tja1145_set_sleep_WUP_CAN(spi);
}

/*
 * tja1145_configure_wake_can_standardId
 */
static void tja1145_configure_wake_can_standardId( struct spi_device *spi, struct can_filter *wu_settings)
{
	dev_dbg(&spi->dev, "%s ---> Id: %x Mask: %x\n", __func__, wu_settings->can_id, wu_settings->can_mask  );
	//Enable_TJA1145_CAN_WUP
	tja1145_write_single_reg(spi, REG_IDENTIFIER_2,	((wu_settings->can_id << 2)   & 0xfc) );
	tja1145_write_single_reg(spi, REG_IDENTIFIER_3,	((wu_settings->can_id >> 6)	  & 0x1f) );
	tja1145_write_single_reg(spi, REG_MASK_2,	((wu_settings->can_mask << 2) & 0xfc) );
	tja1145_write_single_reg(spi, REG_MASK_3,	((wu_settings->can_mask >> 6) & 0x1f) );
	tja1145_write_single_reg(spi, REG_FRAME_CONTROL, 0x00 );
	//Set_TJA1145_Sleep_WUP_CAN
	tja1145_set_sleep_WUP_CAN(spi);
}

/*
 * tja1145_configure_wake_can
 */
static void tja1145_configure_wake_can( struct spi_device *spi, struct can_filter *wu_settings)
{
	dev_dbg(&spi->dev, "%s ---> %x:%x \n", __func__, wu_settings->can_id, wu_settings->can_mask );

	dev_info(&spi->dev, "Setup WakeUp over CAN %s Id 0x%x:0x%x \n", \
			 (wu_settings->can_id & CAN_EFF_FLAG) ? "Extended" : "Standard", \
			 (wu_settings->can_id & CAN_EFF_MASK), wu_settings->can_mask );

	if(wu_settings->can_id &  CAN_EFF_FLAG)
		tja1145_configure_wake_can_extendedId(spi, wu_settings);
	else
		tja1145_configure_wake_can_standardId(spi, wu_settings);
}

/*
 * tja1145_set_working_normal_mode
 */
static void tja1145_set_working_normal_mode( struct spi_device *spi )
{
	dev_dbg(&spi->dev, "%s \n", __func__ );
	tja1145_get_status_and_clear(spi);
	tja1145_write_single_reg(spi, REG_MODE_CONTROL, NORMAL_MODE );
	tja1145_write_single_reg(spi, REG_CAN_CONTROL,  (PNCOK_INVALID | CPNC_DISABLED | CMC_ACTIVE_NO_OVERV) );
	/* Clean regs */
	tja1145_write_single_reg(spi, REG_TRANSCEIVER_EVENT_ENABLE,	0x00 ); //CWE = 0, CFE=0, CBSE=0
	tja1145_write_single_reg(spi, REG_FRAME_CONTROL,		0x00 );
	tja1145_write_single_reg(spi, REG_IDENTIFIER_0,			0x00 );
	tja1145_write_single_reg(spi, REG_IDENTIFIER_1,			0x00 );
	tja1145_write_single_reg(spi, REG_IDENTIFIER_2,			0x00 );
	tja1145_write_single_reg(spi, REG_IDENTIFIER_3,			0x00 );
	tja1145_write_single_reg(spi, REG_MASK_0,			0x00 );
	tja1145_write_single_reg(spi, REG_MASK_1,			0x00 );
	tja1145_write_single_reg(spi, REG_MASK_2,			0x00 );
	tja1145_write_single_reg(spi, REG_MASK_3,			0x00 );
	tja1145_write_single_reg(spi, REG_WAKE_PIN_ENABLE,		0x00 );
	tja1145_get_status_and_clear(spi);
}

/*
 * tja1145_ioctl
 */
static void tja1145_ioctl( struct tja1145_functions_accessor *facc, struct ifreq *ifr, int cmd )
{
	struct tja1145_data* data = container_of(facc, struct tja1145_data, func_acc);
	struct spi_device*   spi  = data->spi;
	//struct can_filter __user *cf = ifr->ifr_ifru.ifru_data;
	struct can_filter __user *cf = ifr->ifr_data;
	struct can_filter wu_settings;

	dev_dbg(&spi->dev, "%s cmd: 0X%04X\n", __func__, cmd );
	switch (cmd) {
		case SIOCTJA1145SETWAKEUP:
			dev_dbg(&spi->dev, "%s SIOCTJA1145SETWAKEUP\n", __func__ );
			if( !copy_from_user(&wu_settings, cf, sizeof(wu_settings)) )
				tja1145_configure_wake_can(spi, &wu_settings);
			break;
		case SIOCTJA1145DISWAKEUP:
			dev_dbg(&spi->dev, "%s SIOCTJA1145DISWAKEUP\n", __func__ );
			tja1145_configure_wake_can_disable(spi);
			break;
		case SIOCTJA1145NORMALMODE:
			dev_dbg(&spi->dev, "%s SIOCTJA1145NORMALMODE\n", __func__ );
			tja1145_set_working_normal_mode(spi);
			break;
		case SIOCTJA1145DUMPREGS:
			dev_dbg(&spi->dev, "%s SIOCTJA1145DUMPREGS\n", __func__ );
			tja1145_dump_all_regs(spi);
			break;

		default:
			dev_err(&spi->dev, "%s Unrecognzed ioctl request\n", __func__ );
			break;
	}
}

/*
 * tja1145_change_bitrate
 */
static void tja1145_change_bitrate( struct tja1145_functions_accessor *facc, __u32 bitrate )
{
	struct tja1145_data* data = container_of(facc, struct tja1145_data, func_acc);
	struct spi_device*   spi  = data->spi;
	dev_dbg(&spi->dev, "%s %d \n", __func__, bitrate );

	switch (bitrate) {
	case 50000:
		tja1145_write_single_reg(spi, REG_DATA_RATE, CANSPEED_50KHZ );
		dev_dbg(&spi->dev, "%s: bitrate %d RegVal %x\n", __func__, bitrate, CANSPEED_50KHZ );
		break;
	case 100000:
		tja1145_write_single_reg(spi, REG_DATA_RATE, CANSPEED_100KHZ );
		dev_dbg(&spi->dev, "%s: bitrate %d RegVal %x\n", __func__, bitrate, CANSPEED_100KHZ );
		break;
	case 125000:
		tja1145_write_single_reg(spi, REG_DATA_RATE, CANSPEED_125KHZ );
		dev_dbg(&spi->dev, "%s: bitrate %d RegVal %x\n", __func__, bitrate, CANSPEED_125KHZ );
		break;
	case 250000:
		tja1145_write_single_reg(spi, REG_DATA_RATE, CANSPEED_250KHZ );
		dev_dbg(&spi->dev, "%s: bitrate %d RegVal %x\n", __func__, bitrate, CANSPEED_250KHZ );
		break;
	case 500000:
		tja1145_write_single_reg(spi, REG_DATA_RATE, CANSPEED_500KHZ );
		dev_dbg(&spi->dev, "%s: bitrate %d RegVal %x\n", __func__, bitrate, CANSPEED_500KHZ );
		break;
	case 1000000:
		tja1145_write_single_reg(spi, REG_DATA_RATE, CANSPEED_1MHZ );
		dev_dbg(&spi->dev, "%s: bitrate %d RegVal %x\n", __func__, bitrate, CANSPEED_1MHZ );
		break;
	default:
		dev_err(&spi->dev, "%s Bitrate value not valid %d\n", __func__, bitrate );
		break;
	}

	dev_dbg(&spi->dev, "%s %d STOPPPPPP\n", __func__, bitrate );

}

/*
 * tja1145_set_sleep_mode
 */
static void tja1145_stop_working( struct tja1145_functions_accessor *facc )
{
	struct tja1145_data* data = container_of(facc, struct tja1145_data, func_acc);
	struct spi_device*   spi  = data->spi;
	dev_dbg(&spi->dev, "%s \n", __func__ );
	tja1145_configure_wake_can_disable(spi);
}

/*
 * tja1145_start_working
 */
static void tja1145_start_working( struct tja1145_functions_accessor *facc )
{
	struct tja1145_data* data = container_of(facc, struct tja1145_data, func_acc);
	struct spi_device*   spi  = data->spi;
	dev_dbg(&spi->dev, "%s \n", __func__ );
	tja1145_set_working_normal_mode(spi);
}

/*
 * tja1145_probe
 */
static int tja1145_probe(struct spi_device *spi)
{
	int status = 0;
	int tmp;
	struct device_node	*np     = spi->dev.of_node;
	struct tja1145_data	*data   = NULL;

	dev_info(&spi->dev, "Start probing \n");
	if (!spi->dev.platform_data)
	{
		if (!np){
			dev_err(&spi->dev, "Probing Error: no chip in DT\n");
			return -ENODEV;
		}
	} else {
		dev_err(&spi->dev, "Probing Error: not implemented platform_data\n");
		return -ENODEV;
	}

	tmp = tja1145_get_chip_id(spi);
	if(tmp < 0)
	{
		dev_err(&spi->dev, "Probing Error: chip not compatible\n");
		return -EPROBE_DEFER;
	}

	data = devm_kzalloc(&spi->dev, sizeof(struct tja1145_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->spi = spi_dev_get(spi);
	spi_set_drvdata(spi, data);
	data->func_acc.transceiver_start		= tja1145_start_working;
	data->func_acc.transceiver_stop			= tja1145_stop_working;
	data->func_acc.transceiver_change_bitrate	= tja1145_change_bitrate;
	data->func_acc.transceiver_ioctl		= tja1145_ioctl;
	dev_info(&spi->dev, "Probed\n");
	return status;
}

/*
 * tja1145_remove
 */
static int tja1145_remove(struct spi_device *spi)
{
	dev_info(&spi->dev, "%s \n", __func__ );
	return 0;
}

/*
 * tja1145_command
 */
static int tja1145_command(struct spi_device *client, unsigned int cmd, void *arg)
{
	struct tja1145_data	*data;
	const struct tja1145_functions_accessor **faccp;

	/* only supporting a single command */
	if (cmd != SPI_TJA1145_GET_FUNC_ACCESSOR)
		return -ENOTSUPP;

	/* rudimentary check */
	if (arg == NULL)
		return -EINVAL;

	data = spi_get_drvdata(client);

	faccp = arg;
	*faccp = &data->func_acc;

	return 0;
}

/*
 * tja1145_dt_ids
 */
static const struct of_device_id tja1145_dt_ids[] = {
{ .compatible = "nxp,tja1145" },
{},
};
MODULE_DEVICE_TABLE(of, tja1145_dt_ids);

/*
 * tja1145_spi_driver
 */
static struct spi_driver tja1145_spi_driver = {
	.driver = {
		.name =		"tja1145",
		.owner =	THIS_MODULE,
		.of_match_table = of_match_ptr(tja1145_dt_ids),
	},
	.probe      = tja1145_probe,
	.remove     = tja1145_remove,
	.id_table	= tja1145_id,
	.command    = tja1145_command,
};

module_spi_driver(tja1145_spi_driver);


MODULE_AUTHOR("Luigi Scagnet <luigi.scagnet@exorint.it>, ");
MODULE_DESCRIPTION("NXP TJA1145 CAN transceiver driver");
MODULE_LICENSE("GPL");
