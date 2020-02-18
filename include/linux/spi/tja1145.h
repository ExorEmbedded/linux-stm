/*
 *  tja1145.h - Linux kernel module for uSxx working hours counters
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
 */
#ifndef SPI_TJA1145_H
#define SPI_TJA1145_H

#include	<linux/types.h>
#include	<linux/memory.h>
#include	<linux/err.h>
#include	<linux/spi/spi.h>
#include	<linux/can.h>

#define		MAJOR_DRIVER_VERS			1
#define		MINOR_DRIVER_VERS			4
/***********************************/
/****** TJA1145 SPI Registers ******/
/***********************************/
/* SPI REG ACCESS */
#define	READ_ONLY					(0x01)
#define	WRITE_ONLY					(0x00)
/* REG ADDR */
#define	REG_MODE_CONTROL				(0x01)
#define	REG_MAIN_STATUS					(0x03)
#define	REG_SYSTEM_EVENT_ENABLE				(0x04)
#define	REG_MEMORY_0					(0x06)
#define	REG_MEMORY_1					(0x07)
#define	REG_MEMORY_2					(0x08)
#define	REG_MEMORY_3					(0x09)
#define	REG_LOCK_CONTROL				(0x0A)

#define	REG_CAN_CONTROL					(0x20)
#define	REG_TRANSCEIVER_STATUS				(0x22)
#define	REG_TRANSCEIVER_EVENT_ENABLE			(0x23)
#define	REG_DATA_RATE					(0x26)
#define	REG_IDENTIFIER_0				(0x27)
#define	REG_IDENTIFIER_1				(0x28)
#define	REG_IDENTIFIER_2				(0x29)
#define	REG_IDENTIFIER_3				(0x2A)
#define	REG_MASK_0					(0x2B)
#define	REG_MASK_1					(0x2C)
#define	REG_MASK_2					(0x2D)
#define	REG_MASK_3					(0x2E)
#define	REG_FRAME_CONTROL				(0x2F)

#define	REG_DATA_MASK0					(0x68)
#define	REG_DATA_MASK1					(0x69)
#define	REG_DATA_MASK2					(0x6A)
#define	REG_DATA_MASK3					(0x6B)
#define	REG_DATA_MASK4					(0x6C)
#define	REG_DATA_MASK5					(0x6D)
#define	REG_DATA_MASK6					(0x6E)
#define	REG_DATA_MASK7					(0x6F)

#define	REG_WAKE_PIN_STATUS				(0x4B)
#define	REG_WAKE_PIN_ENABLE				(0x4C)

#define	REG_EVENT_CAPTURE_STATUS			(0x60)
#define	REG_SYSTEM_EVENT_STATUS				(0x61)
#define	REG_TRANSCEIVER_EVENT_STATUS			(0x63)
#define	REG_WAKE_EVENT_STATUS				(0x64)

#define	REG_IDENTIFICATION				(0x7E)

/* REG VALUE            */
/*  REG_MODE_CONTROL    */
#define	OFFLINE_MODE					(0x00)
#define	SLEEP_MODE					(0x01)
#define	STANDBY_MODE					(0x04)
#define	NORMAL_MODE					(0x07)
#define	NMS_SET						(0x00)	/** Normal Mode Status = Normal Mode */
#define	NMS_NOT_SET					(0x01)	/** Transceiver has powered up but has not yet switched to Normal Mode */
#define	PNCOK_INVALID					(0x00)	/** CAN partial networking disabled */
#define	CPNC_DISABLED					(0x00)	/** CAN selective wake-up disabled */
#define	CMC_ACTIVE_NO_OVERV				(0x02)	/** Active mode, no Vcc undervoltage detection	*/
/*  REG_IDENTIFICATION  */
#define	WHOIAM_VALUE					(0x70)
#define	WHOIAM_VALUE_FD					(0x74)

/* REG_FRAME_CONTROL */
#define	EXTENDED_FRAME_FORMAT				(0x80)
#define	DLC_DATA_WAKEUP					(0X40)
#define	DLC_8						(0x08)
#define	DLC_7						(0x07)
#define	DLC_6						(0x06)
#define	DLC_5						(0x05)
#define	DLC_4						(0x04)
#define	DLC_3						(0x03)
#define	DLC_2						(0x02)
#define	DLC_1						(0x01)
#define	DLC_0						(0x00)

/* interface commands */
#define	SPI_TJA1145_GET_FUNC_ACCESSOR			1

/*
 * Define IOCTL
 */
enum {
	SIOCTJA1145SETWAKEUP = SIOCDEVPRIVATE,		/** 0x89F0 */
	SIOCTJA1145DISWAKEUP,				/** 0x89F1 */
	SIOCTJA1145NORMALMODE,				/** 0x89F2 */
	SIOCTJA1145DUMPREGS,				/** 0x89F3 */
};

typedef enum {
	CANSPEED_50KHZ	= 0,	//0
	CANSPEED_100KHZ,	//1
	CANSPEED_125KHZ,	//2
	CANSPEED_250KHZ,	//3
	CANSPEED_500KHZ	= 5,	//5
	CANSPEED_1MHZ	= 7,	//7
} CANSpeed_t;

/*
 * The method called in the client is
 *
 * int (*command)(struct spi_device *client, unsigned int cmd, void *arg);
 *
 * A single command is supported, which returns a pointer to the memory
 * accessor already available, but which was only accessible via platform
 * callbacks. We can't use platform callbacks anymore for device tree
 * platforms, hence this interface.
 *
 */

struct tja1145_functions_accessor {
	void	(*transceiver_start)		( struct tja1145_functions_accessor *fc );
	void	(*transceiver_stop)		( struct tja1145_functions_accessor *fc );
	void	(*transceiver_change_bitrate)	( struct tja1145_functions_accessor *fc, __u32 bitrate );
	void	(*transceiver_ioctl)		( struct tja1145_functions_accessor *fc, struct ifreq *ifr, int cmd );
};

static inline struct tja1145_functions_accessor *
		spi_tja1145_get_func_accessor(struct spi_device *client)
{
	int ret;
	struct tja1145_functions_accessor *fc;
	struct spi_driver *driver;

	/* verify that the i2c client's driver has a command method */
	if (!client || !client->dev.driver)
		return ERR_PTR(-ENOTSUPP);

	driver = to_spi_driver(client->dev.driver);
	if(!driver->command)
		return ERR_PTR(-ENOTSUPP);

	fc = NULL;
	ret = driver->command(client, SPI_TJA1145_GET_FUNC_ACCESSOR, &fc);
	if (ret != 0)
		return ERR_PTR(ret);

	if (fc == NULL)
		return ERR_PTR(-ENOTSUPP);

	return fc;
}

/*
 * Guard function, used to create a dependency btw flexcan and tja
 */
void tja1145_driver_version(struct spi_device*   spi);

#endif
