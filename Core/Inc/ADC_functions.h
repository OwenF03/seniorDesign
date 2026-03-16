/*
 * ADC_functions.h
 *
 *  Created on: Mar 5, 2026
 *      Author: owen
 */

#ifndef INC_ADC_FUNCTIONS_H_
#define INC_ADC_FUNCTIONS_H_

//Convert adc reading into voltage (stored into res array of floats)
void adc_voltage(float * res, uint16_t * data, size_t len);
// Callback runs when adcData is full
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);

// Return true if signal is valid (contains non noise components)
//
// @param data : Sample to inspect
bool energy_detector(uint16_t * sample);

// Return true if all sensors ADC samples are valid
//
// On MCU m_adc will be a global snapshot of ADC values (of size NUMSENSORS * numSnapshots)
bool valid_adc_sample();

#endif /* INC_ADC_FUNCTIONS_H_ */
