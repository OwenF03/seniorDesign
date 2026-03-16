#include "user_includes.h"

//Global Variables
extern volatile State next_state_i; // Defined in main.cpp
extern volatile State state; // Defined in main.cpp

// Callback runs when adcData is full
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	//Data acquired, change state depending on the current state
	if(state == CALIB){ // In calibration logic
		next_state_i = CALIB_gathered; // Calibration data gathered
	}
	else if(state == DT){ // Waiting for data to transfer
		next_state_i = DT_buf; //Data is ready to transfer
	}
	else if(state == MUSIC_gather){ //MUSIC waiting for data
		next_state_i = MUSIC_validate; // Validate data
	}

}

//Convert adc value into voltage, in place
void adc_voltage(float * res, uint16_t * data,  size_t len){
	for(unsigned int i = 0; i < len; i++){
		*(res + i) = ((float) *(data + i)) * ADC_VOLT; //Convert adc value to voltage
	}
}

// Return true if signal is valid (contains non noise components)
//
// @param data : Sample to inspect
volatile int valid_threshold = 80000; // Configurable threshold via commands, defaults to 80k
bool energy_detector(uint16_t * sample){
    int mean = 0;
    int sum = 0;
    for(int i = 0; i < (numSnapshots); i++){
        mean += ((int)sample[i]);
    }
    mean = mean / (numSnapshots);
    //std::cout << "MEAN : " << mean << "\n";
    for(int i = 0; i < (numSnapshots); i++){
        sum += ((int)sample[i] - mean) * ((int)sample[i] - mean);
    }
    //std::cout << "SUM : " << sum << "\n";
    return (sum >= valid_threshold);
}


// Determine if sample is valid by calculating the energy of the signal
// Operates on voltage data in voltage global array
extern volatile uint16_t adcData[];
extern volatile float voltage[];
extern volatile uint16_t adcData[];
// Return true if all sensors ADC samples are valid
//
// On MCU m_adc will be a global snapshot of ADC values (of size NUMSENSORS * numSnapshots)
bool valid_adc_sample(){
	// Gather ADC values from each sensor into sub arrays
	uint16_t adc_vals[NUMSENSORS][numSnapshots];
	for(int i = 0; i < numSnapshots * NUMSENSORS; i++){
		adc_vals[i % 2][i / NUMSENSORS] = adcData[i];
	}
	bool all_valid = true;
	for(int i = 0; i < NUMSENSORS; i++){
		all_valid = energy_detector(adc_vals[i]);
	}
	return all_valid;
}


