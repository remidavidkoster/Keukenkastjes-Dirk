/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stm32g4xx_it.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


#include "Biquad.h"





/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */



//uint32_t timeDiv[500];
//uint32_t time[500];
//uint32_t timeDiv2[500];
//uint32_t timeDivIndex;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


enum {
	WAITING_FOR_THRESHOLD,
	WAITING_FOR_PEAK,
	DECLINING,
	WAITING_FOR_SILENCE
};

uint32_t detecionState = 0;


#define DETECTION_THRESHOLD 1000
#define RELEASE_THRESHOLD 500
#define ERROR_THRESHOLD 10
#define MIN_SLOPE 2

uint16_t lastClapTimestamp, time, firstTime, secondTime;
uint8_t clapped, lightState;

enum {
	gotFirstClap,
	gotSecondClap
};

uint8_t clapState = gotFirstClap;

// Units in 0.1ms
#define minClapTime 1000
#define maxClapTime 8000


float cosFast(float x){
    x *= 0.159154943f;
    x -= 0.25f + (int)(x + 0.25f);
    x *= 16.0f * (ABS(x) - 0.5f);
    return x;
}

#define PI 3.14159265359f




uint16_t ledValue[8];

void setLedsFromBuffer(){
	TIM1->CCR1 = ledValue[0];
	TIM1->CCR2 = ledValue[1];
	TIM1->CCR3 = ledValue[2];
	TIM1->CCR4 = ledValue[3];
	TIM3->CCR1 = ledValue[4];
	TIM3->CCR2 = ledValue[5];
	TIM4->CCR1 = ledValue[6];
	TIM4->CCR2 = ledValue[7];
}





// Cosine ramp from 0 to 1
float flatCos(float x){
    return 0.5f - 0.5f * (cosFast(PI * x));
}

#define PWM_MAX 65535

void fadeIn(){
	for (int i = 0; i <= 250; i++){

		float fader = flatCos(i / 250.0f);
		uint16_t value = fader * fader * PWM_MAX;

		for (int led = 0; led < 8; led++) ledValue[led] = value;

		setLedsFromBuffer();

		for (uint16_t timer = TIM17->CNT; TIM17->CNT - timer < 100;);
	}
}



void fadeOut(){
	for (int i = 250; i >= 0; i--){

		float fader = flatCos(i / 250.0f);
		uint16_t value = fader * fader * PWM_MAX;

		for (int led = 0; led < 8; led++) ledValue[led] = value;

		setLedsFromBuffer();

		for (uint16_t timer = TIM17->CNT; TIM17->CNT - timer < 100;);
	}
}




void fadeInSequential(){

	int32_t counter = 0;

	while (TIM4->CCR2 < PWM_MAX){



		for (int led = 0; led < 8; led++) {


			float fader = flatCos(LIMIT(0, counter - led * 50, 250) / 250.0f);
			uint16_t value = fader * fader * PWM_MAX;

			ledValue[led] = value;
		}

		setLedsFromBuffer();


		for (uint16_t timer = TIM17->CNT; TIM17->CNT - timer < 100;);

		counter += 1;
	}
	asm("nop");

}




void fadeOutSequential(){

	int32_t counter = 391;

	while (TIM4->CCR2 > 0){

		for (int led = 0; led < 8; led++) {
			float fader = flatCos(LIMIT(0, counter - (7-led) * 50, 250) / 250.0f);
			uint16_t value = fader * fader * PWM_MAX;

			ledValue[led] = value;
		}

		setLedsFromBuffer();


		for (uint16_t timer = TIM17->CNT; TIM17->CNT - timer < 100;);

		counter -= 1;
	}

	asm("nop");

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
	MX_ADC1_Init();
	MX_DAC1_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM8_Init();
	MX_USART2_UART_Init();
	MX_TIM2_Init();
	MX_TIM17_Init();
	/* USER CODE BEGIN 2 */

	// Start timer 8 for ADC trigger
	HAL_TIM_Base_Start(&htim8);

	// Start timer 2 for time keeping
	HAL_TIM_Base_Start(&htim2);

	// Start timer 2 for time keeping [ms]
	HAL_TIM_Base_Start(&htim17);

	// Start ADC with interrupt
	HAL_ADC_Start_IT(&hadc1);

	// Start DACs for monitoring
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

	// Initialise data, right aligned 12 bit
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

	TIM1->CCR1 = 1000;
	TIM1->CCR2 = 1000;
	TIM1->CCR3 = 1000;
	TIM1->CCR4 = 1000;

	TIM3->CCR1 = 1000;
	TIM3->CCR2 = 1000;

	TIM4->CCR1 = 1000;
	TIM4->CCR2 = 1000;


	// Stop HAL systick bullshit
	  HAL_SuspendTick();

	uint32_t errorCounter = 0;
	uint16_t lastData = 0;


	fadeInSequential();
	lightState = 1;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		if (newValue){

			// Reset new data flag
			newValue = 0;

			// Update waveform to DAC
			DAC1->DHR12R1 = newData / 3.3f * 4.096f;



			/// Pre-handle overflows
			if (TIM17->CNT - lastClapTimestamp > 20000){
				lastClapTimestamp = TIM17->CNT - 20000;

			}



			if (detecionState == WAITING_FOR_THRESHOLD){

				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);

				if (newData >= DETECTION_THRESHOLD){
					detecionState = WAITING_FOR_PEAK;
				}
			}



			else if (detecionState == WAITING_FOR_PEAK){

				DAC1->DHR12R2 = 500 / 3.3f * 4.096f;

				if (newData < lastData){

					time = TIM17->CNT - lastClapTimestamp;

					// Save timestamp
					lastClapTimestamp = TIM17->CNT;

					detecionState = DECLINING;
				}
			}



			else if (detecionState == DECLINING){

				DAC1->DHR12R2 = 1000 / 3.3f * 4.096f;


				// If it fucks up
				if (newData > lastData - MIN_SLOPE){
					errorCounter++;

					if (errorCounter >= ERROR_THRESHOLD){
						detecionState = WAITING_FOR_SILENCE;
						errorCounter = 0;
					}
				}


				// Here it made it to the end
				if (newData < RELEASE_THRESHOLD){

					clapped = 1;

					detecionState = WAITING_FOR_SILENCE;
					//				  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 3000 / 3.3f * 4.096f);
					DAC1->DHR12R2 = 3000 / 3.3f * 4.096f;

					HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
					for (uint16_t time = TIM17->CNT; TIM17->CNT - time < 1;);
					HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

				}

			}



			else if (detecionState == WAITING_FOR_SILENCE){
				DAC1->DHR12R2 = 1500 / 3.3f * 4.096f;

				if (newData == 0){
					detecionState = WAITING_FOR_THRESHOLD;

				}
			}

			// Save data for next round
			lastData = newData;

		}








		if (clapped){
			clapped = 0;

			if (clapState == gotFirstClap){
				if (time > minClapTime && time < maxClapTime){
					firstTime = time;
					clapState = gotSecondClap;
//					Serial.print("Got second clap. Time = ");
//					Serial.println(firstTime);
				}
				else {
					clapState = gotFirstClap;
//					Serial.println("Got first clap.");
				}
			}

			else if (clapState == gotSecondClap){
				if (time > minClapTime && time < maxClapTime){
					secondTime = time;
					clapState = gotFirstClap;
//					Serial.print("Got third clap. Time = ");
//					Serial.println(secondTime);
					if (firstTime > secondTime * 0.25f && firstTime < secondTime * 0.75f){
//						Serial.println("Full On");
//						checkPassed(0);
//						mode = clapValue;
//						data[1] = 1023;
						HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
						if (lightState != 1) {
							fadeInSequential();
							lightState = 1;
						}
						else if (lightState != 0) {
							fadeOutSequential();
							lightState = 0;
						}

					}
//					if (secondTime > firstTime * 0.25f && secondTime < firstTime * 0.75f){
////						Serial.println("Full off");
////						checkPassed(0);
////						mode = clapValue;
////						data[1] = 0;
//						HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//
//
//					}
//					if (secondTime > firstTime * 0.75f && secondTime < firstTime * 1.25f){
////						Serial.println("Full off");
////						checkPassed(0);
////						mode = clapValue;
////						data[1] = 0;
//						HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//						lightState = 2;
//					}
				}
				else {
					clapState = gotFirstClap;
//					Serial.println("Got first clap.");
				}
			}
		}





		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
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

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the peripherals clocks
	 */
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC12;
	PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
