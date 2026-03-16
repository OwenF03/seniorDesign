/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "user_includes.h"


// Global State Variables
volatile State state = INVALID; //Default state to be invalid
volatile State next_state = INVALID; //Save recieved command
volatile State next_state_a = INVALID; // "next state action", used for when next state is ack
volatile State next_state_i = INVALID;// Next state from interrupt
volatile State next_state_r = INVALID; // Recieve updates state
volatile State next_state_post_ack = INVALID;

volatile uint16_t adcData[DATALEN]; //Store raw adc values
volatile float voltage[DATALEN]; //store converted values
volatile float tx_buffer[DT_MAX_SIZE * NUMSENSORS]; // Store data to be transmitted over UART

float fc[N_signals];
float spacing;
volatile uint8_t rx_cmd[CMD_LEN];
uint32_t transmitLength = 0;

// Flags for controlling state
volatile int calibrating = 0;

volatile float m_voltage[numSnapshots * NUMSENSORS]; // Store voltage sample for MUSIC

std::vector<std::vector<struct Peak>> resultBuffer; // Store MUSIC results
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  HAL_Init();
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_SPI1_Init();
  MX_TIM5_Init();

  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t * ) adcData, DATALEN);
  HAL_TIM_Base_Start(&htim8); //Timer running at 400 KHz, should trigger a DMA sample every 400 KSPS
  HAL_UART_Receive_IT(&huart2, (uint8_t * )rx_cmd, CMD_LEN); //Check for command to start transmion

  disableCSlines();

  //Set initial gain to 200
  gain_init();
  f_MUSIC_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {
	  //Switch based on state
	  if(state == CALIB_gathered){
		  next_state = calibrate(adcData);
	  }
	  else if(state == IDLE || state == wait_T1){
		  HAL_UART_Receive_IT(&huart2, (uint8_t *) rx_cmd, CMD_LEN); // Listen for next command
		  next_state = IDLE;
	  }
	  else if(state == DT_ready){
		  next_state = f_DT_ready();
	  }
	  else if(state == DT_buf){
		  next_state = f_DT_buf();
	  }
	  else if(state == DT_transmitting || state == ACK_t || state == DT || state == MUSIC_t || state == MUSIC_gather){
		  next_state = state; //Wait for tx complete callback to change state, or for ADC to collect data
		  	  	  	  	  	  // Wait for data to become available.
	  }
	  else if (state == SPI_t){
		  //Retrieve values from command buffer
		  // No validation, but channel won't go out of bounds regardless of value
		  //send_SPI(rx_cmd[3], rx_cmd[4]);
		  send_SPI(rx_cmd[3], rx_cmd[4]);
		  next_state = IDLE; // Done sending SPI signal
	  }
	  else if (state == INVALID){
		  next_state = IDLE; //Likely sending a response is not necessary
	  }
	  else if(state == START){ //Start DOA processing
		  HAL_UART_Receive_IT(&huart2, (uint8_t *) rx_cmd, CMD_LEN); // Listen for next command
		  resultBuffer.clear(); //Clear buffer
		  next_state = MUSIC_gather;
	  }
	  else if(state == MUSIC_gather){
		  // Wait for ADC interrupt
	  }
	  else if(state == MUSIC_validate){
		  next_state = f_MUSIC_validate();
	  }
	  else if(state == MUSIC_process){
		  next_state = f_MUSIC_process();
	  }
	  else if(state == MUSIC_transmit){
		  next_state = f_MUSIC_transmit();
	  }
	  else if(state == STOP){
		  resultBuffer.clear(); //Clear DOA buffer
		  next_state = IDLE; //Stop music calculations
	  }
	  else if (state == ACK){
		  next_state = f_send_ACK();
	  } else{
		  next_state = INVALID;
	  }

	  // state transitions
	  if(next_state_r != INVALID){ // First priority to received command
		  if((state == DT_transmitting) || (state == MUSIC_t) || (state == ACK_t)){ // Don't update state while these commands are active
			  //
		  }else{
			  state = next_state_r;
			  next_state_r = INVALID; //Clear state
		  }

	  }else if (next_state_i != INVALID){ //Second priority is interrupt state update
		  if(next_state_r == INVALID){
			  state = next_state_i;
		  }
		  else{
			  state = next_state_r;
		  }
		  next_state_r = INVALID;
		  next_state_i = INVALID;
	  }
	  else{ // Main logic updated state
		  state = next_state;
	  }

  }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
