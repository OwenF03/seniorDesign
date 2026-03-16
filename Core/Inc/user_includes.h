
#ifndef INC_USER_INCLUDES_H_
#define INC_USER_INCLUDES_H_

// Auto generated files
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#define CMD_LEN 8
#define NUMSENSORS 4
#define NUMSAMPLES numSnapshots
#define DT_MAX_SIZE 1024		// Must be >= NUMSAMPLES
#define DATALEN NUMSENSORS * NUMSAMPLES
#define ADC_VOLT 0.000805860805861f
#define MUSIC_BUFFER 4

#define CAL "cal"
#define STA "sta"
#define STP "stp"
#define DAT "dat"
#define SFR "sfr"
#define SPI "spi"
#define SNL "snl"
#define SPA "spa"

// TODO : Should probs be an enum
//#define st_CALIB 0
//#define st_CALIB_gathered 5
//#define st_CALIB_proc 6
//#define st_START 1
//#define st_STOP 2
//#define st_DT 3
//#define st_INVALID 4
//#define st_IDLE 7
//#define st_DT_ready 8
//#define st_DT_transmitting 9
//#define st_ACK 10

enum State{
	CALIB, CALIB_gathered, CALIB_proc, START, STOP, DT,
	INVALID, IDLE, DT_ready, DT_transmitting, ACK, SPI_t,
	MUSIC_gather, MUSIC_validate, MUSIC_process, MUSIC_transmit, MUSIC_invalid, MUSIC_t,
	testTrans1, testTrans2, wait_T1, ACK_t, DT_buf

};


// Other includes
// TODO: THe position of these includes does matter (must be after state), consider resolving
#include <MUSIC.h>
//#include "input_data.h" // TODO : Currently an ODR violation, move to .cpp file
#include "gain_control.h"
#include "main.h"
#include "params.h"
#include "data_transfer.h"
#include "MUSIC_state.h"
#include "ADC_functions.h"

void SystemClock_Config(void); // Declare, used by main.cpp

#endif /* INC_USER_INCLUDES_H_ */
