/*
 * FIR Coefficients for processing
 *
 * STM32-SDR: A software defined HAM radio embedded system.
 * Copyright (C) 2013, STM32-SDR Group
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include "arm_math.h"
#include "FIR_Coefficients.h"

// 32kBaseband

//////////////////////////////////////////////////////////////////////
q15_t coeff_fir_I_32K[NUM_FIR_COEF] = {0, 52, 14, -42, -99, -140, -147, -116,
									   -56, 8, 46, 28, -57, -195, -347, -461, -490, -415, -261, -93, 5, -40,
									   -252, -589, -949, -1199, -1235, -1022, -628, -208, 44, -21, -446, -1125,
									   -1830, -2284, -2272, -1737, -830, 124, 733, 697, -62, -1330, -2642,
									   -3441, -3284, -2042, 0, 2190, 3718, 3908, 2518, -86, -2950, -4769,
									   -4292, -762, 5747, 14222, 22900, 29693, 32767, 31065, 24645, 14692,
									   3227, -7438, -15302, -19178, -18947, -15492, -10360, -5261, -1577, -13,
									   -501, -2343, -4531, -6122, -6538, -5705, -4001, -2063, -526, 197, 33,
									   -770, -1772, -2525, -2734, -2341, -1518, -573, 180, 531, 445, 43, -456,
									   -831, -939, -758, -375, 58, 392, 533, 470, 267, 25, -155, -214, -149, 0,
									   164, 284, 324, 283, 190, 86, 7, -24, -6, 44, 103, 147, 164, 149, 0

};
//////////////////////////////////////////////////////////////////////
q15_t coeff_fir_Q_32K[NUM_FIR_COEF] = {0, -112, -150, -165, -148, -104, -45, 5,
									   23, -8, -87, -191, -284, -325, -285, -165, 0, 148, 213, 154, -26, -268,
									   -471, -534, -393, -59, 374, 757, 938, 830, 455, -44, -446, -532, -181,
									   572, 1517, 2340, 2733, 2524, 1771, 769, -34, -198, 525, 2062, 4000,
									   5704, 6537, 6121, 4530, 2342, 500, 12, 1576, 5260, 10359, 15491, 18946,
									   19177, 15301, 7437, -3228, -14693, -24646, -31066, -32767, -29694,
									   -22901, -14223, -5748, 761, 4291, 4768, 2949, 85, -2519, -3909, -3719,
									   -2191, 0, 2041, 3283, 3440, 2641, 1329, 61, -698, -734, -125, 829, 1736,
									   2271, 2283, 1829, 1124, 445, 20, -45, 207, 627, 1021, 1234, 1198, 948,
									   588, 251, 39, -6, 92, 260, 414, 489, 460, 346, 194, 56, -29, -47, -9,
									   55, 115, 146, 139, 98, 41, -15, 0};
