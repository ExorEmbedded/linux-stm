/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2018 STMicroelectronics
 * Author(s): Amelie Delaunay <amelie.delaunay@st.com>.
 */

#ifndef MFD_STMFX_H
#define MFX_STMFX_H

#include <linux/regmap.h>

enum stmfx_functions {
	STMFX_FUNC_GPIO		= BIT(0), /* GPIO[15:0] */
	STMFX_FUNC_ALTGPIO_LOW	= BIT(1), /* aGPIO[3:0] */
	STMFX_FUNC_ALTGPIO_HIGH = BIT(2), /* aGPIO[7:4] */
	STMFX_FUNC_TS		= BIT(3),
	STMFX_FUNC_IDD		= BIT(4),
};

struct stmfx {
	struct device *dev;
	struct regmap *map;
};

int stmfx_function_enable(struct stmfx *stmfx, u32 func);
int stmfx_function_disable(struct stmfx *stmfx, u32 func);
#endif
