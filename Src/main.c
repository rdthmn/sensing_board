/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "board.h"
#include "arm_math.h"
#include "sx1272.h"

/* ----------------------------------------------------------------------
** Macro Defines
** ------------------------------------------------------------------- */
#define OFF						0
#define	VIBE_IN_PROGRESS		1
#define	VIBE_DONE				2
#define	VIBE_FFT				3
#define GET_TEMP				4
#define MAKE_PACKET				5
#define SEND_PACKET				6

/* ----------------------------------------------------------------------
** Global Variables
** ------------------------------------------------------------------- */
uint8_t measureMode = OFF;
uint16_t sample = 0;
uint8_t tempOutput[3];
extern uint8_t vibrationOutput[VIBE_SIZE];
uint8_t payload[PAYLOAD_LENGTH], payload0[32];
uint8_t flags = 0;
uint16_t crc = 0;

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

SX1272 node;

Config_Group config;

/* Private function prototypes -----------------------------------------------*/
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void process_measurements(void);
static void MX_SPI1_Init(void);
void log_temp(void);
float32_t convert_to_temp(uint8_t *tsVal);
void set_payload(void);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {
  /* MCU Configuration--------------------------------------------------------*/

	HAL_Init();
	BRD_init();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_SPI1_Init();
	ADC1_Init();

	node.hspi = &hspi1;
	node.config = &config;

	sx1272_lora_init(&node);

	  /* Dummy data */
	payload0[0] = 0x55;
	payload0[1] = 0x17;
	payload0[2] = 0x22;
	payload0[3] = 0x10;
	payload0[4] = 0x34;
	payload0[5] = 0x89;
	payload0[6] = 0xA3;
	payload0[7] = 0x5F;
	for (int i = 8; i < 32; i++) {
		payload0[i] = 0;
	}

	/* Infinite loop */
	while (1) {
		process_measurements();

		/* Heartbeat*/
		BRD_delay(500);
		BRD_led_toggle();
	}
}

/**
  * @brief  Processes steps to capture all measurements
  * @param  None
  * @retval None
  */
void process_measurements(void) {

	/* Initialise timer for vibration sensor */
	if (measureMode == OFF) {
		debug_printf("START\n\r");
		TIM2_Init();
		measureMode = VIBE_IN_PROGRESS;
		return;
	}
	/* Capturing vibrations */
	if (measureMode == VIBE_IN_PROGRESS) {
		return;
	}
	/* Deinitialise timer for vibration sensor */
	if (measureMode == VIBE_DONE) {
		TIM2_Deinit();
		measureMode = VIBE_FFT;
		return;
	}
	/* Filter and FFT of captured data */
	if (measureMode == VIBE_FFT) {
		vibe_filter();
		vibe_fft();
		measureMode = GET_TEMP;
		return;
	}

	/* Capture temperature data */
	if (measureMode == GET_TEMP) {
		log_temp();
		measureMode = MAKE_PACKET;
		return;
	}

	/* Set up payload to send */
	if (measureMode == MAKE_PACKET) {
		set_payload();
		measureMode = SEND_PACKET;

		debug_printf("\n\r");
		for (int i = 0; i < (PAYLOAD_LENGTH); i++){
			debug_printf("%d ", payload[i]);
		}
		return;
	}

	/* Send payload */
	if (measureMode == SEND_PACKET) {
		flags = sx1272_send(0x5, payload0, 32 + HEADER_LENGTH, 1, 100);
		measureMode = OFF;
	}

	debug_printf("\n\r\n\rDONE\n\r\n\r");
	BRD_button_unpush();
	return;
}

void log_temp(void) {

	uint8_t ts[2];

	HAL_I2C_Mem_Read(&hi2c1, 0x51 << 1, 0x7, 1, ts, 2, 100);
	tempOutput[0] = convert_to_temp(ts);

	HAL_I2C_Mem_Read(&hi2c1, 0x52 << 1, 0x7, 1, ts, 2, 100);
	tempOutput[1] = convert_to_temp(ts);

	HAL_I2C_Mem_Read(&hi2c1, 0x53 << 1, 0x7, 1, ts, 2, 100);
	tempOutput[2] = convert_to_temp(ts);
}

float32_t convert_to_temp(uint8_t *tsVal) {

	float32_t temp = 0.0;

	temp = (tsVal[1] << 8) | tsVal[0];
	temp = temp * 0.02 - 273.15;

	return temp;
}


/**
* @brief Sets up payload to be sent
* @param None
* @retval None
*/
void set_payload(void) {

	/*Pack Temperature data*/
	for (int i = 0; i < NUM_TEMP_SENSORS; i++) {
		payload[i] = tempOutput[i];
	}

	/*Pack vibration data*/
	for(int i = 0; i < VIBE_SIZE; i++) {
		payload[i + NUM_TEMP_SENSORS] = vibrationOutput[i];
	}

}


/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {

  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {

  }
}





/**
* @brief Period elapsed callback in non blocking mode
* @param htim: Pointer to a TIM_HandleTypeDef that contains the configuration information for the TIM module.
* @retval None
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		/* End ADC sampling */
		if (sample >= SAMPLES) {
			measureMode = VIBE_DONE;
			sample = 0;
			return;
		}

		if (measureMode != VIBE_DONE) {
			/* Capture ADC samples */
			ADC_fill_buffer(sample);
			sample++;
		}
	}
}


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct;

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : NSS_Pin */
  GPIO_InitStruct.Pin = NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(NSS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
