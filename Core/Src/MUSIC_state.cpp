/*
 * MUSIC_state.cpp
 *
 *  Created on: Mar 5, 2026
 *      Author: owen
 */
#include "MUSIC_state.h"
// Function to calculate DOA of incoming signal using the MUSIC Direction finding algorithm
int musicBufIdx = 0;

// Should configure single object rather then construct in process function, tbi
extern float fc[];
extern float spacing;
extern std::vector<std::vector<struct Peak>> resultBuffer;
DOA estimator;
void f_MUSIC_init(){
	// Set result buffer size for DOA calculations
  resultBuffer.reserve(MUSIC_BUFFER);
  for(int i = 0; i < N_signals; i++){ // Default fc to 30000
	  fc[i] = 30000;
  }
  spacing = elSpacing;
  estimator = DOA(400000, spacing, fc); //Create DOA estimator object

}

//Determine if ADC data is valid
State f_MUSIC_validate(){
	// Perform processing
	bool good_sample = valid_adc_sample();
	if(!good_sample){
		return MUSIC_gather; // Collect another sample
	}
	else{
		return MUSIC_process;
	}
}


// Perform DOA calculation
extern float m_voltage[];
State f_MUSIC_process(){
	// Sample validated, perform calculation
	//memcpy((void *) data.data(), (void *) input_data, input_data_len); // Load Test data
	// Real call would be
	auto res = estimator.estimateDOA(m_voltage); // Calculate DOA
	resultBuffer.emplace_back(res); // Store into buffer
	if(resultBuffer.size() >= MUSIC_BUFFER){ // Buffer full send result over UART
		return MUSIC_transmit;
	} else{
		return MUSIC_gather; // Process more results
	}
}

extern volatile float tx_buffer[];
// Transmit DOA values
State f_MUSIC_transmit(){
	char resp[3 + 2 * N_signals] = "RES";
	// Average results
	std::vector<short> res(N_signals);
	for(auto i : resultBuffer){
	  for(int j = 0; j < N_signals; j++){
		  res[j] += i[j].idx - 90;
	  }
	}
	int i = res[0];
	int j = res[1];
	for(int i = 0; i < N_signals; i++){
	  res[i] /= MUSIC_BUFFER;
	}

	for(int i = 0; i < N_signals; i+=1){
	  resp[2 * i + 3] = res[i] & 0xFF;
	  resp[2 * i + 4] = (res[i] >> 8) & 0xFF;
	}

	memcpy((void *) tx_buffer, (uint8_t *) resp, 3 + 2 * N_signals);
	HAL_Delay(100);
	if(HAL_UART_Transmit_DMA(&huart2, (uint8_t *) tx_buffer, 2 * N_signals + 3)){
	  return START;
	}
	return MUSIC_t;

}

