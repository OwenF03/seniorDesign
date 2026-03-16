#include "data_transfer.h"


// Global Variables Used / Modified
extern float spacing; // MUSIC element spacing
extern float fc[]; // MUSIC center frequencies
extern uint32_t transmitLength; // In samples
extern volatile State next_state_a;
extern volatile State next_state_r;
extern volatile State next_state_post_ack;
extern volatile State next_state_i;
extern volatile State state;
extern volatile float voltage[];
extern volatile float tx_buffer[];
extern volatile uint16_t adcData[];
extern volatile uint8_t rx_cmd[];
extern volatile int valid_threshold;

uint32_t dt_buf_pos = 0;

// ADC data is available to send
State f_DT_ready(){

	  HAL_Delay(10); //Delay for PySerial to work
	  if(HAL_UART_Transmit_DMA(&huart2, (uint8_t *) tx_buffer, transmitLength *  sizeof(float)) != HAL_OK){
		  dt_buf_pos = 0; // Reset buffer
		  return DT; //Error occurred
	  }
	  dt_buf_pos = 0; // Reset buffer
	  return DT_transmitting; //Indicate that transmission has started
}


// Function which manages Data Transfer Buffer
State f_DT_buf(){
	adc_voltage((float *)voltage, (uint16_t *) adcData, DATALEN); //Convert to voltage
	int read_in = transmitLength - dt_buf_pos;
	if(read_in > DATALEN) read_in = DATALEN;
	memcpy((void *) (tx_buffer + dt_buf_pos), (float *) voltage, read_in * sizeof(float)); //copy in data to buf
	dt_buf_pos+=read_in; // Iterate buffer
	if(dt_buf_pos == transmitLength){
		return DT_ready; // Buffer full, transmit
	}
	else{
		return DT; // Collect more data
	}
}

// Called when a cmd (8 bytes) is received
// This sets a flag that is handled in the main while loop to delay transmission
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	// TODO : Break out into parse command function?
	char cmd[4] = {(char) (rx_cmd[0] | 32), (char) (rx_cmd[1] | 32), (char) (rx_cmd[2] | 32), '\0'};

	if(!strcmp(cmd, CAL)){ // Perform calibration
		next_state_a = CALIB;
	}else if (!strcmp(cmd, STA)){ // Start performing DOA calculations on ADC data
		next_state_a = START;
	}else if (!strcmp(cmd, STP)){ // Stop performing DOA calculations on ADC data
		 next_state_a = STOP;
	}else if (!strcmp(cmd, DAT)){ // Send block of ADC data over UART (slow, need better protocol to stream ADC)
		transmitLength = rx_cmd[6] << 24 | rx_cmd[5] << 16 | rx_cmd[4] << 8 | rx_cmd[3]; //Get number of samples to send
		if(transmitLength > DT_MAX_SIZE) transmitLength = DT_MAX_SIZE;
		// Convert into number of samples to collect
		transmitLength = NUMSENSORS * transmitLength;
		next_state_a = DT;
	}else if (!strcmp(cmd, SFR)){ // Set target frequency for DOA algorithm
		for(int i = 1; i <= N_signals; i++){ // Read center frequencies
			fc[i] = rx_cmd[3 * i + 3] << 24 | rx_cmd[3 * i + 2] << 16 | rx_cmd[3 * i + 1] << 8 | rx_cmd[3 * i];
		}
		next_state_a = IDLE;
	}else if(!strcmp(cmd, SPA)){
		spacing = rx_cmd[6] << 24 | rx_cmd[5] << 16 | rx_cmd[4] << 8 | rx_cmd[3];
		next_state_a = IDLE;
	}else if(!strcmp(cmd, SNL)){
		// To be implemented
		next_state_a = IDLE;
	}
	else if (!strcmp(cmd, SPI)){ //Send SPI value over channel
		// TODO : define variables for SPI channel and value rather then use rx_cmd?
		next_state_a = SPI_t; //SPI transfer
	}else if (!strcmp(cmd, "ech"))
		next_state_a = testTrans1;

	else{ // Invalid Command Received
		next_state_a = IDLE;
	}

	next_state_r = ACK; // Send confirmation that command was received

}

// Run when UART transmit has completed
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(state == DT_transmitting){
		next_state_i = IDLE;
	}else if(state == ACK_t){
		next_state_i = next_state_post_ack;
	}else if(state == MUSIC_t){
		next_state_i = START; // Perform another music calculation
	}
}

State f_send_ACK(){
	if(HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX) {
		return ACK; // Try again
    }
    // Create response
    char *resp = "ACK\0\0\0\0\0";
    if(next_state_a == IDLE) resp = "BADCMD\0\0";
    for(int i = 0; i < CMD_LEN; i++) *((uint8_t*)(tx_buffer) + i) = *(resp + i);
    HAL_Delay(10);
    next_state_post_ack = next_state_a; // Prevent race condition in tx_complete
    if(HAL_UART_Transmit_DMA(&huart2, (uint8_t *) tx_buffer, CMD_LEN)){
	  return  ACK;
    }
    return ACK_t;
}
