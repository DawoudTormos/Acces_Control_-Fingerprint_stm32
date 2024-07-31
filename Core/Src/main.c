/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "fp_interface.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CORRECT_PASS_FLASH 2
#define INCORRECT_PASS_FLASH 2

#define CORRECT_FP_FLASH 2
#define INCORRECT_FP_FLASH 2

#define FP_SAVED_FLASH 5
#define FP_NOT_SAVED_FLASH 5

#define ERASE_UNSUCESS_FLASH 3

#define LOCK_OPENED_FLASH 1
#define LOCK_CLOSED_FLASH 1

#define FLASH_DELAY 300 // in ms

#define MOTOR_RUNTIME 5000 // in ms
#define LOCK_RUNTIME 10000 // 2 min -- in ms
#define delay_to_close 10000 // in ms

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t pkt_received = 0; // variable to store if there is a new packet from bluetooth
uint8_t fp_correct = 2; // variable to store if the entered fp is correct (1: correct, 0: incorrect)
uint8_t motor_state = 0; // variable to store the state of the motor
uint8_t lock_state = 0; // variable to store the state of the lock
uint8_t password_correct = 0; // variable to store if the entered password is correct

char *password = "1234";
uint32_t motor_on_since= 0 ;
uint32_t fp_tobeClosed_since= 0 ;
uint32_t fp_tobeClosed = 0 ;
int x = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
	fp_init();

	uint8_t bluetooth_data[strlen(password)];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */










		HAL_UART_Receive_IT(&huart2, bluetooth_data, sizeof(bluetooth_data)); // Enabling interrupt receive

		// if a new bluetooth packet is received
		if (pkt_received == 1) {
			pkt_received = 0;

			// check if the entered password bytes are equal to the password saved
			uint8_t count_corr = 0;
			for (int i = 0; i < strlen(password); i++) {
				if (bluetooth_data[i] == (uint8_t) password[i]) {
					count_corr++;
				}
			}

			// it the password is entered correctly
			if (count_corr == strlen(password)) {

				flush_buff();
				uint8_t fp_deleted = PS_Empty(); // empty the saved fingerprints

				// if FPs are deleted successfully
				if (fp_deleted == 0) {
					password_correct = 1; // enable the save of a new FP

					HAL_GPIO_WritePin(LOCK_GPIO_Port, LOCK_Pin, 1); // open the lock
					HAL_GPIO_WritePin(YELLOW_GPIO_Port, YELLOW_Pin, 1); // Yellow is ON
					HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, 0); // Green is OFF
					HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, 0); // Red is OFF
				} else {
					for (int i = 0; i < ERASE_UNSUCESS_FLASH; i++) {
						HAL_GPIO_WritePin(YELLOW_GPIO_Port, YELLOW_Pin, 1); // Yellow is ON
						HAL_Delay(FLASH_DELAY);
						HAL_GPIO_WritePin(YELLOW_GPIO_Port, YELLOW_Pin, 0); // Yellow is ON
						HAL_Delay(FLASH_DELAY);
					}
				}

			} else {
				password_correct = 0;

				for (int i = 0; i < INCORRECT_PASS_FLASH; i++) {
					HAL_GPIO_WritePin(YELLOW_GPIO_Port, YELLOW_Pin, 1); // Yellow is ON
					HAL_Delay(FLASH_DELAY);
					HAL_GPIO_WritePin(YELLOW_GPIO_Port, YELLOW_Pin, 0); // Yellow is ON
					HAL_Delay(FLASH_DELAY);
				}
			}

			memset(bluetooth_data, '\0', sizeof(bluetooth_data));
		}













		uint32_t currentTime = HAL_GetTick();


		// save the state of the touch sensor
		uint8_t touch_sens_state = HAL_GPIO_ReadPin(TOUCH_SENS_GPIO_Port,
		TOUCH_SENS_Pin);

		// if the touch sensor is enabled
		// run the motor for 1 sec



		if (touch_sens_state) {
			currentTime = HAL_GetTick();
			for(;HAL_GetTick() - currentTime <5;){
				if(HAL_GPIO_ReadPin(TOUCH_SENS_GPIO_Port,TOUCH_SENS_Pin)){

				}else{
					touch_sens_state = 0;
					break;
				}
			}
		}//To be certain the touch sensor was touched. Debugging a problem with false touch inputs




		currentTime = HAL_GetTick();
		if (touch_sens_state) {

			fp_tobeClosed = 0;
			if(!motor_state){
			HAL_GPIO_WritePin(MD_IN1_GPIO_Port, MD_IN1_Pin, 1);
			HAL_GPIO_WritePin(MD_IN2_GPIO_Port, MD_IN2_Pin, 0);
			motor_state = 1;
			motor_on_since = currentTime;


			}else{
				if(currentTime - motor_on_since > MOTOR_RUNTIME){
				HAL_GPIO_WritePin(MD_IN1_GPIO_Port, MD_IN1_Pin, 0);
				HAL_GPIO_WritePin(MD_IN2_GPIO_Port, MD_IN2_Pin, 0);
				}


			}



			//HAL_Delay(MOTOR_RUNTIME); // wait 1 sec

			// Stop the motor



			// Run the motor in a specific direction

		} else {
			if(fp_tobeClosed == 0){
				fp_tobeClosed_since = currentTime;
				fp_tobeClosed = 1;
			}
			if(currentTime - fp_tobeClosed_since > delay_to_close){
			if (motor_state == 1) {
				motor_state = 0;

				// Run the motor in the opposite direction
				HAL_GPIO_WritePin(MD_IN1_GPIO_Port, MD_IN1_Pin, 0);
				HAL_GPIO_WritePin(MD_IN2_GPIO_Port, MD_IN2_Pin, 1);

				HAL_Delay(MOTOR_RUNTIME); // wait 1 sec

				// Stop the motor
				HAL_GPIO_WritePin(MD_IN1_GPIO_Port, MD_IN1_Pin, 0);
				HAL_GPIO_WritePin(MD_IN2_GPIO_Port, MD_IN2_Pin, 0);
			}
			}



		}











		uint8_t usb_data[9];
		uint32_t enroll;
		uint32_t score;
		uint16_t temp_buff_id;

		// if a fingerprint is detected
		if (is_fp_detected() == 1) {

			temp_buff_id = PS_ValidTempleteNum(); // read the number of saved fingerprints
			int get_image = PS_GetImage(); // get the image (returns: ok: 00, messy, dry, wet)

			flush_buff();





			// if the image acquisition is successful and the password is entered correctly
			// save the FP
			if (get_image == 0 && password_correct) {// here it is detecting the fingerprint and saving it after the user has entered the password through bluwtooth




				// is required to put your finger on the FP 2 times for saving
				for (int i = 0; i < 2; i++) {
					enroll = PS_AutoEnroll(temp_buff_id, 2, 0x003F); // save the fingerprint
				}

				// if FP is saved
				if (((enroll >> 16) & 0xFF) == 0) {
					password_correct = 0; // reset the password correct flag
					HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, 0); // Red is OFF
					HAL_GPIO_WritePin(YELLOW_GPIO_Port, YELLOW_Pin, 0); // Yellow is OFF

					// flash the green multiple times to indicate successful fp save
					for (int i = 0; i < FP_SAVED_FLASH; i++) {
						HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, 1);
						HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, 1);
						HAL_Delay(FLASH_DELAY);
						HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, 0);
						HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, 0);
						HAL_Delay(FLASH_DELAY);
					}
				} else {
					for (int i = 0; i < FP_NOT_SAVED_FLASH; i++) {
						HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, 1);
						HAL_Delay(FLASH_DELAY);
						HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, 0);
						HAL_Delay(FLASH_DELAY);
					}
				}
				flush_buff();






			} else if (get_image == 0) {




				for (int i = 0; i < temp_buff_id; i++) {
					score = PS_AutoIdentify(i, 20);

					// if the score of the fp is high
					if (((score >> 8 | score) & 0xFF) > 3) {
						fp_correct = 1;
						break;
					} else {
						fp_correct = 0;
					}
					HAL_Delay(3);
				}





			} else if (get_image != 0 ) {

				if(x>0){HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, 1);}//remooving initilly red error
				flush_buff();
				x++;
			}





			usb_data[0] = temp_buff_id >> 8; // valid fp template high byte
			usb_data[1] = temp_buff_id; // valid fp template low byte
			usb_data[2] = get_image; // conf code of get image get_image

			usb_data[3] = score >> 24; // conf code of enroll
			usb_data[4] = score >> 16; // parameter 1
			usb_data[5] = score >> 8; // // parameter 2
			usb_data[6] = score; // conf code of enroll

			usb_data[7] = enroll >> 8; // parameter 1
			usb_data[8] = enroll; // // parameter 2

			HAL_Delay(50);
			CDC_Transmit_FS(usb_data, sizeof(usb_data));
			clear_fp_flag();





		}// end of is_fp_detected












		if (fp_correct == 1) {

			static uint32_t lock_run_time = 0;

			HAL_GPIO_WritePin(LOCK_GPIO_Port, LOCK_Pin, 1);
			HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, 1);
			HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, 0);

			if (lock_run_time == 0)
				lock_run_time = HAL_GetTick();

			if (HAL_GetTick() - lock_run_time > LOCK_RUNTIME) {
				HAL_GPIO_WritePin(LOCK_GPIO_Port, LOCK_Pin, 0);
				HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, 0);
				lock_run_time = 0;
				fp_correct = 2;
			}
		} else if (fp_correct == 0 ) {
			static uint32_t lock_run_time = 0;

			HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, 1);

			if (lock_run_time == 0)
				lock_run_time = HAL_GetTick();

			if (HAL_GetTick() - lock_run_time > LOCK_RUNTIME) {
				HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, 0);
				lock_run_time = 0;
				fp_correct = 2;
			}

		}






		HAL_Delay(40);







	}//infinite while loop end here

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 57600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MD_EN_Pin|MD_IN2_Pin|MD_IN1_Pin|RED_Pin
                          |YELLOW_Pin|GREEN_Pin|LOCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TOUCH_SENS_Pin */
  GPIO_InitStruct.Pin = TOUCH_SENS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TOUCH_SENS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MD_EN_Pin MD_IN2_Pin MD_IN1_Pin RED_Pin
                           YELLOW_Pin GREEN_Pin LOCK_Pin */
  GPIO_InitStruct.Pin = MD_EN_Pin|MD_IN2_Pin|MD_IN1_Pin|RED_Pin
                          |YELLOW_Pin|GREEN_Pin|LOCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : FP_IRQ_Pin */
  GPIO_InitStruct.Pin = FP_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FP_IRQ_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		pkt_received = 1;
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	}
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {

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
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
