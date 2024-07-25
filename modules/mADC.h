/*
 * @project EPFL-HXL_PS_v1.0
 * @file    mADC.h
 * @brief   Author:             MBE
 *          Institute:          EPFL
 *          Laboratory:         LMTS
 *          Firmware version:   v1.09
 *          Created on:         12.02.2024
 *          Last modifications: 22.03.2024
 *
 * HXL_PS © 2021-2024 by MBE
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the GNU Lesser 
 * General Public License as published by the Free Software Foundation, either version 3 of the License, or 
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the 
 * implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public 
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this library.  If not, see:
 * <http://www.gnu.org/licenses/>.
 *
 * NO HELP WILL BE GIVEN IF YOU MODIFY THIS CODE !!!
 */

#ifndef MADC_H_
#define MADC_H_

//Personal Includes
#include "source/globals.h"

class mADC {
public:
	// Differential conversion for ADC0
	static void adc0_df_setup();
	static uint16_t adc0_df_run(uint32_t UserChannel);
	// Single-ended conversion for ADC0
	static void adc0_se_setup();
	static uint16_t adc0_se_run(uint32_t UserChannel);
	// Single-ended conversion for ADC1
	static void adc1_se_setup();
	static uint16_t adc1_se_run(uint32_t UserChannel);
};

#endif /* MADC_H_ */
