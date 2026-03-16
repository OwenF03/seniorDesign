#ifndef INC_GAIN_CONTROL_H_
#define INC_GAIN_CONTROL_H_
#include "user_includes.h"
#include "main.h"


void gain_init();
// Check if a certain threshold of samples are saturated
// window = Number of samples to be checked (max of NUMSAMPLES)
// thresh = what counts as saturated
// Stores recommended gain correction in corrections
//	-5 for > 70 % saturated
//  -2 for > 40 % saturated
//  -1 for > 5 % saturated (aiming for ~5% saturated in a noisy env)
// Returns 1 if any corrective value is non zero
int checkForSat(int window, int thresh, int * corrections, volatile uint16_t * adcData);


// Zero chip select lines
void disableCSlines();

//Calibrate gain on LNA boards
// Return state
State calibrate(volatile uint16_t * adcData);

void send_SPI(uint8_t val, uint8_t channel);

void m_send_SPI(uint8_t val, uint8_t channel);



#endif /* INC_GAIN_CONTROL_H_ */
