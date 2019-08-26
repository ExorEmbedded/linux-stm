/* SPDX-License-Identifier: GPL-2.0 */
/*
 * This file is part of STM32 ADC driver
 *
 * Copyright (C) 2016, STMicroelectronics - All Rights Reserved
 * Author: Fabrice Gasnier <fabrice.gasnier@st.com>.
 *
 */

#ifndef __STM32_ADC_H
#define __STM32_ADC_H

/*
 * STM32 - ADC global register map
 * ________________________________________________________
 * | Offset |                 Register                    |
 * --------------------------------------------------------
 * | 0x000  |                Master ADC1                  |
 * --------------------------------------------------------
 * | 0x100  |                Slave ADC2                   |
 * --------------------------------------------------------
 * | 0x200  |                Slave ADC3                   |
 * --------------------------------------------------------
 * | 0x300  |         Master & Slave common regs          |
 * --------------------------------------------------------
 */
#define STM32_ADC_MAX_ADCS		3
#define STM32_ADC_OFFSET		0x100
#define STM32_ADCX_COMN_OFFSET		0x300

/* Number of linear calibration shadow registers / LINCALRDYW control bits */
#define STM32H7_LINCALFACT_NUM		6

/**
 * struct stm32_adc_calib - optional adc calibration data
 * @calfact_s: Calibration offset for single ended channels
 * @calfact_d: Calibration offset in differential
 * @lincalfact: Linearity calibration factor
 * @calibrated: Indicates calibration status
 */
struct stm32_adc_calib {
	u32			calfact_s;
	u32			calfact_d;
	u32			lincalfact[STM32H7_LINCALFACT_NUM];
	bool			calibrated;
};

/* STM32F4 - common registers for all ADC instances: 1, 2 & 3 */
#define STM32F4_ADC_CCR			(STM32_ADCX_COMN_OFFSET + 0x04)

/* STM32F4_ADC_CCR - bit fields */
#define STM32F4_ADC_TSVREFE		BIT(23)

/* STM32H7 - common registers for all ADC instances */
#define STM32H7_ADC_CCR			(STM32_ADCX_COMN_OFFSET + 0x08)

/* STM32H7_ADC_CCR - bit fields */
#define STM32H7_VSENSEEN		BIT(23)

/**
 * struct stm32_adc_common - stm32 ADC driver common data (for all instances)
 * @base:		control registers base cpu addr
 * @phys_base:		control registers base physical addr
 * @rate:		clock rate used for analog circuitry
 * @vref_mv:		vref voltage (mv)
 * @extrig_list:	External trigger list registered by adc core
 *
 * Reserved variables for child devices, shared between regular/injected:
 * @difsel		bitmask array to set single-ended/differential channel
 * @pcsel		bitmask array to preselect channels on some devices
 * @smpr_val:		sampling time settings (e.g. smpr1 / smpr2)
 */
struct stm32_adc_common {
	void __iomem			*base;
	phys_addr_t			phys_base;
	unsigned long			rate;
	int				vref_mv;
	struct list_head		extrig_list;
	u32				difsel[STM32_ADC_MAX_ADCS];
	u32				pcsel[STM32_ADC_MAX_ADCS];
	u32				smpr_val[STM32_ADC_MAX_ADCS][2];
	int				prepcnt[STM32_ADC_MAX_ADCS];
	struct mutex			inj[STM32_ADC_MAX_ADCS]; /* injected */
	struct stm32_adc_calib		cal[STM32_ADC_MAX_ADCS];
};

/* extsel - trigger mux selection value */
enum stm32_adc_extsel {
	STM32_EXT0,
	STM32_EXT1,
	STM32_EXT2,
	STM32_EXT3,
	STM32_EXT4,
	STM32_EXT5,
	STM32_EXT6,
	STM32_EXT7,
	STM32_EXT8,
	STM32_EXT9,
	STM32_EXT10,
	STM32_EXT11,
	STM32_EXT12,
	STM32_EXT13,
	STM32_EXT14,
	STM32_EXT15,
	STM32_EXT16,
	STM32_EXT17,
	STM32_EXT18,
	STM32_EXT19,
	STM32_EXT20,
};

/* trigger information flags */
#define TRG_REGULAR	BIT(0)
#define TRG_INJECTED	BIT(1)
#define TRG_BOTH	(TRG_REGULAR | TRG_INJECTED)

/**
 * struct stm32_adc_trig_info - ADC trigger info
 * @name:		name of the trigger, corresponding to its source
 * @extsel:		regular trigger selection
 * @jextsel:		injected trigger selection
 * @flags:		trigger flags: e.g. for regular, injected or both
 */
struct stm32_adc_trig_info {
	const char *name;
	u32 extsel;
	u32 jextsel;
	u32 flags;
};

#endif
