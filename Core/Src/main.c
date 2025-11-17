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
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CMD_LEN 8
#define NUMSENSORS 4
#define NUMSAMPLES 512
#define DATALEN NUMSENSORS * NUMSAMPLES
#define ADC_VOLT 0.000805860805861f

int state = -1;
int transmitLength = 0;

#define CALIB "cal"
#define START "sta"
#define STOP "sto"
#define DATA_TRANSFER "dat"
// TODO : Should probs be an enum
#define st_CALIB 0
#define st_CALIB_gathered 5
#define st_CALIB_proc 6
#define st_START 1
#define st_STOP 2
#define st_DT 3
#define st_INVALID 4
#define st_IDLE 7
#define st_DT_ready 8
#define st_DT_transmitting 9

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint16_t adcData[DATALEN]; //Store raw adc values
volatile float voltage[DATALEN]; //store converted values
volatile float tx_buffer[DATALEN]; // Store data to be transmitted over UART

//Convert adc reading into voltage (stored into res array of floats)
void adc_voltage(float * res, uint16_t * data, size_t len);

volatile int timeout = 0;
volatile int cmdTimeout = 0;
volatile uint8_t startTrans = 0;
volatile uint8_t uartComplete = 1;
volatile uint8_t d_rdy = 0;
volatile uint8_t rx_cmd[CMD_LEN];
volatile int transmit = 0;
GPIO_TypeDef * CSports[NUMSENSORS] = {CS0_GPIO_Port, CS1_GPIO_Port, CS2_GPIO_Port, CS3_GPIO_Port};
uint16_t CSpins[NUMSENSORS] = {CS0_Pin, CS1_Pin, CS2_Pin, CS3_Pin};
uint8_t LNA_gain[NUMSENSORS] = {200, 200, 200, 200};

// Flags for controlling state
volatile int calibrating = 0;

// Callback runs when adcData is full
// Converts into voltage
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
	if(state == st_CALIB){
		state = st_CALIB_gathered;
	}
	else if(state == st_DT){
		adc_voltage(voltage, adcData, DATALEN);
		state = st_DT_ready;
	}

}

// Check if a certain threshold of samples are saturated
// window = Number of samples to be checked (max of NUMSAMPLES)
// thresh = what counts as saturated
// Stores recommended gain correction in corrections
//	-5 for > 70 % saturated
//  -2 for > 40 % saturated
//  -1 for > 5 % saturated (aiming for ~5% saturated in a noisy env)
// Returns 1 if any corrective value is non zero
int checkForSat(int window, int thresh, int corrections[NUMSENSORS]){

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



// Zero chip select lines
void disableCSlines(){
	HAL_GPIO_WritePin(CS0_GPIO_Port, CS0_Pin, 1);
	HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 1);
	HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, 1);
	HAL_GPIO_WritePin(CS3_GPIO_Port, CS3_Pin, 1);

}

//Calibrate gain on LNA boards
void calibrate(){

	int corrections[NUMSENSORS] = {};

	if(checkForSat(256, 4000, &corrections)){
		for(int i = 0; i < NUMSENSORS; i++){
			if(corrections[i]){
				LNA_gain[i] += corrections[i];
				disableCSlines();
				HAL_GPIO_WritePin(CSports[i], CSpins[i], 0); //Pull CS  low
				HAL_SPI_Transmit(SPI2_BASE, LNA_gain[i], 1, 30);
			}
		}
		state = st_CALIB;
	}

	state = st_IDLE;


}




//Called when a cmd (8 bytes) is recieved
// Because PySerial doesn't work as it should (imo)
// This sets a flag that is handled in the main while loop to delay transmission
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
		startTrans = 1;
		char cmd[4] = {rx_cmd[0] | 32, rx_cmd[1] | 32,  rx_cmd[2] | 32, '\0'};
		if(!strcmp(cmd, CALIB)){
			state = st_CALIB;
		}else if (!strcmp(cmd, START)){
			state = st_START;
		}else if (!strcmp(cmd, STOP)){
			state = st_STOP;
		}else if (!strcmp(cmd, DATA_TRANSFER)){
			transmitLength = rx_cmd[3] << 24 | rx_cmd[4] << 16 | rx_cmd[5] << 8 | rx_cmd[6]; //Get number of samples to send
			if(transmitLength > NUMSAMPLES) transmitLength = NUMSAMPLES;
			state = st_DT;
		}else{
			state = st_INVALID;
		}
		//HAL_UART_Receive_IT(&huart2, &rx_cmd, CMD_LEN); // Listen again
}

//
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	state = st_IDLE;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t * ) adcData, DATALEN);
  HAL_TIM_Base_Start(&htim8); //Timer running at 400 KHz, should trigger a DMA sample every 400 KSPS
  HAL_UART_Receive_IT(&huart2, &rx_cmd, CMD_LEN); //Check for command to start transmion

  disableCSlines();

  //Set initial gain to 200
  for(int i = 0; i < NUMSENSORS; i++){
	HAL_GPIO_WritePin(CSports[i], CSpins[i], 0); //Pull CS  low
	HAL_SPI_Transmit(SPI2_BASE, LNA_gain[i], 1, 30);

  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //Switch based on state
	  if(state == st_CALIB_gathered){
		  calibrate();
	  }
	  else if(state == st_IDLE){
		  HAL_UART_Receive_IT(&huart2, &rx_cmd, CMD_LEN); // Listen for next command
	  }
	  else if(state == st_DT_ready){
		  memcpy(tx_buffer, voltage, transmitLength * NUMSENSORS * sizeof(float));
		  HAL_Delay(100); //Delay for PySerial to work
		  if(HAL_UART_Transmit_DMA(&huart2, (uint8_t *) tx_buffer, transmitLength * NUMSENSORS * sizeof(float)) != HAL_OK){
			  return -1; //Error occurred, return from main
		  }
		  state = st_DT_transmitting; //Indicate that transmission has started
	  }
	  else if (state == st_INVALID){
		  state = st_IDLE; //Likely sending a response is not necesary
	  }


	  /*
	  if(d_rdy & !transmit){

	  HAL_Delay(100);
	  */

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  int y = 0;

  /* USER CODE END 3 */
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

/* USER CODE BEGIN 4 */
//Convert adc value into voltage, in place
void adc_voltage(float * res, uint16_t * data,  size_t len){
	for(int i = 0; i < len; i++){
		*(res + i) = ((float) *(data + i)) * ADC_VOLT; //Convert adc value to voltage
	}
}
/* USER CODE END 4 */

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
