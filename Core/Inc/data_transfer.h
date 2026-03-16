// Contains functions relating to transfering data between MCU and external device over UART
#ifndef INC_DATA_TRANSFER_H_
#define INC_DATA_TRANSFER_H_
#include "params.h"
#include "user_includes.h"

State f_DT_ready();
State f_DT_buf();
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
State f_send_ACK();

#endif /* INC_DATA_TRANSFER_H_ */
