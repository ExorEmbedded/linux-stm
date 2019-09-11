/*********************************************************************
*
*  Copyright (C) 2014 Exor International, Inc.
*  All Rights Reserved.
*
*  File:			I2CSeeprom.h
*
*  Description:		Contains all definitions regarding the layout of
*					the I2C SEEPROM memory on carrier board containing 
*					the factory parameters.
*  Rev. history:
*  05/05/10:		Created G.Pavoni Exor S.p.a.
*********************************************************************/

#ifndef _I2CSEEPROMFACTORY
#define _I2CSEEPROMFACTORY

// I2C SEEPROM slave address 
#ifndef SEEPROM_I2C_ADDRESS
#define SEEPROM_I2C_ADDRESS         0x54
#endif

#ifndef ADP_I2C_ADDRESS
#define ADP_I2C_ADDRESS             0x56
#endif

#ifndef PL_I2C_ADDRESS
#define PL_I2C_ADDRESS              0x57
#endif

#ifndef LEDDIMM_I2C_ADDRESS		
#define LEDDIMM_I2C_ADDRESS         0x60
#endif

#define SEEPROM_I2C_BAUD_INDEX      (0)
#define SEEPROM_I2C_DEVICE          (OMAP_DEVICE_I2C2)

/*=======================================================================
 * Layout of I2C memory
 *======================================================================= */
#define FACTORY_SECTION_SIZE        128		/* First part of I2C = factory section */
#define FACTORY_SECTION_SIZE_3      64		/* First part of I2C = factory section  (version 3)*/

/* Header */
#define SIGNATURE1_POS              0
#define SIGNATURE2_POS              1
#define VERSION_POS                 2
#define CKSUM_POS                   3

/* Data */
#define SEEPROMID_POS               4
#define DISPID_POS                  5

#define AXLL_POS                    6
#define AXLH_POS                    7
#define AXRL_POS                    8
#define AXRH_POS                    9
#define AXBL_POS                    10
#define AXBH_POS                    11
#define AXTL_POS                    12
#define AXTH_POS                    13
#define GXRESL_POS                  14
#define GXRESH_POS                  15
#define GYRESL_POS                  16
#define GYRESH_POS                  17

#define HWDAY_POS                   18
#define HWMONTH_POS                 19
#define HWYEAR_POS                  20

#define HWSERNUML_POS               21
#define HWSERNUMM_POS               22
#define HWSERNUMH_POS               23

#define MACID0_POS                  24
#define MACID1_POS                  25
#define MACID2_POS                  26
#define MACID3_POS                  27
#define MACID4_POS                  28
#define MACID5_POS                  29

/* pos 30 ... 31 is RFU */
/* pos 32 ... 79 is reserved for protocol flags */
/* pos 80 ... is RFU */

/*=======================================================================
 * I2C pre-defined / fixed values
 *======================================================================= */
#define SIGNATURE1_VAL                      0xaa
#define SIGNATURE2_VAL                      0x55
#define VERSION_VAL                         2
#define SEEPROMID_VAL                       0x00
#define RFU_VAL                             0xff
#define ETOP504_VAL                         103
#define ETOP507_VAL                         104
#define EPALM504_VAL                        106
#define EPALMSHD4_VAL                       107
#define HW01_VAL                            108
#define TWS05_VAL                           109


#define DEFAULT_TCH_VAL                     0
#define AMTUSB_TCH_VAL                      1
#define ZXYUSB_TCH_VAL                      2
#define MXT5_TCH_VAL                        4
#define MXT10_TCH_VAL                       5

/*=======================================================================
 * User info section
 *======================================================================= */
#define BLDIMM_POS                          128
#define CONTRAST_POS                        129

#define BLIGHT_TIME1                        130
#define BLIGHT_TIME2                        131
#define BLIGHT_TIME3                        132
#define BLIGHT_TIME4                        133

#define BACKLIGHT_HOUR_COUNTER_OFFSET_I2C   130
// Format 3 SEEPROM specific define
#define BACKLIGHT_HOUR_CHK_OFFSET_I2C       134
#define SYSTEM_HOUR_COUNTER_OFFSET_I2C      136
#define SYSTEM_HOUR_CHK_OFFSET_I2C          140
#define HWPICKPANELCODE_POS                 4
#define TOUCHID_POS                         21
#define WIFIMACID0_POS                      55
#define WIFIMACID1_POS                      56
#define WIFIMACID2_POS                      57
#define WIFIMACID3_POS                      58
#define WIFIMACID4_POS                      59
#define WIFIMACID5_POS                      60

/*=======================================================================
 * Plugin module factory section
 *======================================================================= */
#define SEE_FORMAT_REV      2          /* Format Revision. If > 1 is 24MHz compatible */
#define SEE_CHKSM_START     4          /* start of area covered by checksum */
#define SEE_NAME_OFF       10          /* module name */
#define SEE_CODE_OFF       26          /* hw code     */
#define SEE_SUBCODE_OFF    27          /* hw subcode  */
#define SEE_XILCODE_OFF    29          /* XIL code    */
#define SEE_XILSUBCODE_OFF 30          /* XIL subcode */
#define SEE_FUNCT_AREA_OFF 36          /* Function Area (bit-wide) */
#define SEE_FACTORYSIZE	   64		   /* Length of the factory section */
#define SEE_MODULENAMELEN  16          /* Length of module name field */
#define SEE_FUNCAREALEN    16          /* Length of function area */

/*=======================================================================
 * Most important bits inside the function flag area
 *======================================================================= */
#define CODESYS_ENABLE_FLAG  0
#define CAN_PHYSICAL_IF_FLAG 1
#define KNX_TPUART_IF_FLAG   2
#define RS422_485_IF_FLAG    3  //PLCM04
#define PLIO03_IO_BRD_FLAG   4
#define PLIO06_IO_BRD_FLAG   5
#define PROFIBUS_DP_FLAG     6  //PLCM06
#define RS232_IF_FLAG        7  //PLCM03
#define NMEA2000_ENABLE_FLAG 8
#define PLIO07_IO_BRD_FLAG   9
#define DUAL_CAN_PHYSICAL_IF_FLAG 10
#define PLIO04_IO_BRD_FLAG   11

#endif /* _I2CSEEPROMFACTORY */
