/*
 * @project EPFL-HXL_PS_v1.0
 * @file    mADC.cpp
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

//Personal Includes
#include "source/globals.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ADC16_CHANNEL_GROUP 0U

/*******************************************************************************
 * Variables
 ******************************************************************************/
ADC_Type *ADC16_BASE;
adc16_config_t adc16ConfigStruct;
adc16_channel_config_t adc16ChannelConfigStruct;
adc16_hardware_average_mode_t adc16HwConfigStruct;

/*******************************************************************************
 * Code
 ******************************************************************************/
// Differential conversion for ADC0
void mADC::adc0_df_setup() {
	/*
	 * adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
	 * adc16ConfigStruct.clockSource = kADC16_ClockSourceAsynchronousClock;
	 * adc16ConfigStruct.enableAsynchronousClock = true;
	 * adc16ConfigStruct.clockDivider = kADC16_ClockDivider8;
	 * adc16ConfigStruct.resolution = kADC16_ResolutionDF16Bit;
	 * adc16ConfigStruct.longSampleMode = kADC16_LongSampleDisabled;
	 * adc16ConfigStruct.enableHighSpeed = false;
	 * adc16ConfigStruct.enableLowPower = false;
	 * adc16ConfigStruct.enableContinuousConversion = false;
	 */
	ADC16_BASE = ADC0;

	ADC16_GetDefaultConfig(&adc16ConfigStruct);

	adc16ConfigStruct.resolution = kADC16_ResolutionDF16Bit;

	ADC16_Init(ADC16_BASE, &adc16ConfigStruct);

	ADC16_EnableHardwareTrigger(ADC0, false); /* Make sure the software trigger is used. */
	ADC16_SetHardwareAverage(ADC0, kADC16_HardwareAverageDisabled);

	adc16ChannelConfigStruct.enableDifferentialConversion = true;
	adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;

	return;
}

uint16_t mADC::adc0_df_run(uint32_t UserChannel) {

	adc0_df_setup();

	adc16ChannelConfigStruct.channelNumber = UserChannel;

	ADC16_SetChannelConfig(ADC0, ADC16_CHANNEL_GROUP,
			&adc16ChannelConfigStruct);

	while (0U
			== (kADC16_ChannelConversionDoneFlag
					& ADC16_GetChannelStatusFlags(ADC0, ADC16_CHANNEL_GROUP))) {
	}

	uint16_t value = ADC16_GetChannelConversionValue(ADC0, ADC16_CHANNEL_GROUP);

	return value;
}

// Single-ended conversion for ADC0
void mADC::adc0_se_setup() {
	/*
	 * adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
	 * adc16ConfigStruct.clockSource = kADC16_ClockSourceAsynchronousClock;
	 * adc16ConfigStruct.enableAsynchronousClock = true;
	 * adc16ConfigStruct.clockDivider = kADC16_ClockDivider8;
	 * adc16ConfigStruct.resolution = kADC16_ResolutionSE12Bit;
	 * adc16ConfigStruct.longSampleMode = kADC16_LongSampleDisabled;
	 * adc16ConfigStruct.enableHighSpeed = false;
	 * adc16ConfigStruct.enableLowPower = false;
	 * adc16ConfigStruct.enableContinuousConversion = false;
	 */
	ADC16_BASE = ADC0;

	ADC16_GetDefaultConfig(&adc16ConfigStruct);

	adc16ConfigStruct.resolution = kADC16_ResolutionSE12Bit;

	ADC16_Init(ADC16_BASE, &adc16ConfigStruct);

	ADC16_EnableHardwareTrigger(ADC0, false); /* Make sure the software trigger is used. */
	ADC16_SetHardwareAverage(ADC0, kADC16_HardwareAverageCount32);

	adc16ChannelConfigStruct.enableDifferentialConversion = false;
	adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;

	return;
}

uint16_t mADC::adc0_se_run(uint32_t UserChannel) {

	adc0_se_setup();

	adc16ChannelConfigStruct.channelNumber = UserChannel;

	ADC16_SetChannelConfig(ADC0, ADC16_CHANNEL_GROUP,
			&adc16ChannelConfigStruct);

	while (0U
			== (kADC16_ChannelConversionDoneFlag
					& ADC16_GetChannelStatusFlags(ADC0, ADC16_CHANNEL_GROUP))) {
	}

	uint16_t value = ADC16_GetChannelConversionValue(ADC0, ADC16_CHANNEL_GROUP);

	return value;
}

// Single-ended conversion for ADC1
void mADC::adc1_se_setup() {
	/*
	 * adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceVref;
	 * adc16ConfigStruct.clockSource = kADC16_ClockSourceAsynchronousClock;
	 * adc16ConfigStruct.enableAsynchronousClock = true;
	 * adc16ConfigStruct.clockDivider = kADC16_ClockDivider8;
	 * adc16ConfigStruct.resolution = kADC16_ResolutionSE12Bit;
	 * adc16ConfigStruct.longSampleMode = kADC16_LongSampleDisabled;
	 * adc16ConfigStruct.enableHighSpeed = false;
	 * adc16ConfigStruct.enableLowPower = false;
	 * adc16ConfigStruct.enableContinuousConversion = false;
	 */
	ADC16_BASE = ADC1;

	ADC16_GetDefaultConfig(&adc16ConfigStruct);

	adc16ConfigStruct.resolution = kADC16_ResolutionSE12Bit;

	ADC16_Init(ADC16_BASE, &adc16ConfigStruct);

	ADC16_EnableHardwareTrigger(ADC1, false); /* Make sure the software trigger is used. */
	ADC16_SetHardwareAverage(ADC1, kADC16_HardwareAverageCount32);

	adc16ChannelConfigStruct.enableDifferentialConversion = false;
	adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;

	return;
}

uint16_t mADC::adc1_se_run(uint32_t UserChannel) {

	adc1_se_setup();

	adc16ChannelConfigStruct.channelNumber = UserChannel;

	ADC16_SetChannelConfig(ADC1, ADC16_CHANNEL_GROUP,
			&adc16ChannelConfigStruct);

	while (0U
			== (kADC16_ChannelConversionDoneFlag
					& ADC16_GetChannelStatusFlags(ADC1, ADC16_CHANNEL_GROUP))) {
	}

	uint16_t value = ADC16_GetChannelConversionValue(ADC1, ADC16_CHANNEL_GROUP);

	return value;
}
