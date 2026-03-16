#include "gain_control.h"
#include "spi.h"
#define NUMSENSORS_FINAL 4

GPIO_TypeDef * CSports[NUMSENSORS_FINAL] = {CS0_GPIO_Port, CS1_GPIO_Port, CS2_GPIO_Port, CS3_GPIO_Port};
uint16_t CSpins[NUMSENSORS_FINAL] = {CS0_Pin, CS1_Pin, CS2_Pin, CS3_Pin};
uint8_t LNA_gain[NUMSENSORS_FINAL] = {200, 200, 200, 200};


//Zero CS lines and set default gain
void gain_init(){
	for(int i = 0; i < NUMSENSORS; i++){
		HAL_GPIO_WritePin(CSports[i], CSpins[i], GPIO_PIN_RESET); //Pull CS  low
		const uint8_t tval = LNA_gain[i];
		HAL_SPI_Transmit((SPI_HandleTypeDef*)SPI2_BASE, &tval, 1, 30);

	  }
}

// Zero chip select lines
void disableCSlines(){
	HAL_GPIO_WritePin(CS0_GPIO_Port, CS0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CS3_GPIO_Port, CS3_Pin, GPIO_PIN_SET);

}

// Check if a certain threshold of samples are saturated
// window = Number of samples to be checked (max of NUMSAMPLES)
// thresh = what counts as saturated
// Stores recommended gain correction in corrections
//	-5 for > 70 % saturated
//  -2 for > 40 % saturated
//  -1 for > 5 % saturated (aiming for ~5% saturated in a noisy env)
// Returns 1 if any corrective value is non zero
int checkForSat(int window, int thresh, int * corrections, volatile uint16_t adcData[]){

	int counts[NUMSENSORS] = {};
	for(int i = 0; i < NUMSENSORS; i++){
		for(int j = 0; j < window; j++){
			if(adcData[4 * j + i] > thresh){
				counts[i]++;
			}
		}
	}

	int t3 = window * 0.7;
	int t2 = window * 0.4;
	int t1 = window * 0.05;
	int rv = 0;
	for(int i = 0; i < NUMSENSORS; i++){
		if(counts[i] > t3){
			corrections[i] = -5;
			rv = 1;
		}
		else if(counts[i] > t2) {
			corrections[i] = -2;
			rv = 1;
		}
		else if(counts[i] > t1) {
			corrections[i] = -1;
			rv = 1;
		}
		else corrections[i] = 0;
	}

	return rv;
}
//Calibrate gain on LNA boards
// Return state
State calibrate(volatile uint16_t * adcData){

	int corrections[NUMSENSORS] = {};

	if(checkForSat(256, 4000, corrections, adcData)){
		for(int i = 0; i < NUMSENSORS; i++){
			if(corrections[i]){
				LNA_gain[i] += corrections[i];
				disableCSlines();
				HAL_GPIO_WritePin(CSports[i], CSpins[i], GPIO_PIN_RESET); //Pull CS  low
				const uint8_t tval = LNA_gain[i];
				HAL_SPI_Transmit((SPI_HandleTypeDef*)SPI1_BASE, &tval, 1, 30);
			}
		}
		return CALIB;
	}

	return IDLE;

}

// Send SPI value to LNA board
void send_SPI(uint8_t val, uint8_t channel){
	disableCSlines(); //Bring all chip select lines high
	HAL_GPIO_WritePin(CSports[channel % NUMSENSORS], CSpins[channel % NUMSENSORS], GPIO_PIN_RESET); // Bring chip select line low
	HAL_SPI_Transmit(&hspi1, &val, 1, 1);
	//HAL_Delay(100);
	HAL_GPIO_WritePin(CSports[channel % NUMSENSORS], CSpins[channel % NUMSENSORS], GPIO_PIN_SET); // Bring CS line high
	HAL_GPIO_WritePin(CSports[channel % NUMSENSORS], CSpins[channel % NUMSENSORS], GPIO_PIN_RESET); // Bring CS line high


}


