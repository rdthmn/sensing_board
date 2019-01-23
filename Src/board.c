/**
 ******************************************************************************
 * @file    research/board.c
 * @author  Radley Scott
 * @date    15012019
 * @brief	Board hardware and peripheral initialisation for use on the sensor
 * 			board.
 *
 * External functions
 *				void BRD_init(void)
 *				void BRD_led_on(void)
 *				void BRD_led_off(void)
 *				void BRD_led_toggle(void)
 *				uint8_t BRD_button_pushed(void)
 *				void BRD_button_unpush(void)
 *				void BRD_debuguart_putc(unsigned char c)
 *				void ADC_init(void)
 *				void ADC_Deinit(void)
 *				uint8_t ADC_get(void)
 *				void ADC_fill_buffer(uint16_t pos)
 *				uint8_t ADC_get_buffer(uint16_t pos)
 *				void TIM2_Init(void)
 *				void TIM2_Deinit(void)
 *				void HAL_Delayus(uint32_t us)
 *				void BRD_delay(int counter)
 *				extern void debug_putc(char c)
 *				extern void debug_flush()
 *				extern unsigned char debug_getc()
 *				extern void debug_rxflush()
 *				extern void debug_printf (const char *fmt, ...)
 ******************************************************************************
 */

/* ----------------------------------------------------------------------
** Includes
** ------------------------------------------------------------------- */
#include "board.h"
#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"
#include "arm_math.h"
#include <stdarg.h>
#include <ctype.h>
#include <stdio.h>
#include <string.h>

/* ----------------------------------------------------------------------
** Private typedef
** ------------------------------------------------------------------- */
UART_HandleTypeDef UartHandle;
ADC_HandleTypeDef 	AdcHandle;
TIM_HandleTypeDef TimHandle;

/* ----------------------------------------------------------------------
** Global Variables
** ------------------------------------------------------------------- */
uint8_t adcBuffer[SAMPLES];
static uint8_t adcValue = 0;
static uint8_t button = NOT_PUSHED;
static uint32_t buttonPressTime = 0;
float32_t fftInput[FFT_SAMPLES];
float32_t fftOutput[FFT_SIZE];
uint8_t vibrationOutput[VIBE_SIZE];
float32_t filtOutput[FFT_SIZE];
float32_t filtInput[FFT_SIZE];
uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = FFT_SIZE/BLOCK_SIZE;
static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
//FIR filter coefficients calculated in MATLAB using fir1(20, 2kHz/10kHz/2)
const float32_t firCoeffs32[NUM_TAPS] = {
		0.0, -0.00212227114882539, -0.00632535399151418, -0.0116118103776210, -0.0123546567489824,
		0.0, 0.0317744975585673, 0.0814359075642177, 0.137493781701943, 0.182125490388735,
		0.199168830106960, 0.182125490388735, 0.137493781701943, 0.0814359075642177, 0.0317744975585673,
		0.0, -0.0123546567489824, -0.0116118103776210, -0.00632535399151418, -0.00212227114882539,
		0.0
};

/* ----------------------------------------------------------------------
** Function Prototypes
** ------------------------------------------------------------------- */
void BRD_led_init(void);
void BRD_push_button_init(void);
static void SystemClock_Config(void);
static void Error_Handler(void);
void EXTI15_10_IRQHandler(void);


/**
  * @brief  Initialise the Nucleo Board -> clock, timers, uart, user button
  * @param  None
  * @retval None
  */
void BRD_init() {

	SystemClock_Config();
	BRD_debuguart_init();
	BRD_led_init();
	BRD_push_button_init();
}


/**
  * @brief  Initialise user LED GPIO
  * @param  None
  * @retval None
  */
void BRD_led_init(void) {

	//Enable GPIOA clock
	LED_ENABLE();

	// Define GPIO
	GPIO_InitTypeDef LED_InitStruct;

	LED_InitStruct.Pin = LED_PIN;
	LED_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	LED_InitStruct.Speed = GPIO_SPEED_FAST;
	LED_InitStruct.Pull = GPIO_PULLUP;

	HAL_GPIO_Init(LED_PORT, &LED_InitStruct);
}


/**
  * @brief  Turn on led
  * @param  None
  * @retval None
  */
void BRD_led_on(void) {

	// GPIO A5 ON
	HAL_GPIO_WritePin(LED_PORT, LED_PIN, 1);
}


/**
  * @brief  Turn off led
  * @param  None
  * @retval None
  */
void BRD_led_off(void) {

	// GPIO A5 OFF
	HAL_GPIO_WritePin(LED_PORT, LED_PIN, 0);
}


/**
  * @brief  Toggle led
  * @param  None
  * @retval None
  */
void BRD_led_toggle() {

	// GPIO A5 toggle
	HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
}


/**
  * @brief  Initialise user button GPIO as external interrupt
  * @param  None
  * @retval None
  */
void BRD_push_button_init(void) {

	GPIO_InitTypeDef Button_InitStruct;

	BUTTON_ENABLE();

	Button_InitStruct.Mode = GPIO_MODE_IT_RISING;
	Button_InitStruct.Pull = GPIO_NOPULL;
	Button_InitStruct.Pin  = BUTTON_PIN;
	HAL_GPIO_Init(BUTTON_PORT, &Button_InitStruct);

	HAL_NVIC_SetPriority(BUTTON_EXTI_IRQn, 10, 0);
	HAL_NVIC_EnableIRQ(BUTTON_EXTI_IRQn);

}


/**
  * @brief  Check if button has been pushed
  * @param  None
  * @retval Returns 1 if pushed, else 0
  */
uint8_t BRD_button_pushed(void) {

	if (button == PUSHED) {
		return PUSHED;
	}
	return NOT_PUSHED;
}


/**
  * @brief  Set button to unpushed state (0)
  * @param  None
  * @retval None
  */
void BRD_button_unpush(void) {

	button = NOT_PUSHED;
}


/**
  * @brief  User button external interrupt handler
  * @param  GPIO_Pin: Pointer to the pin that contains the configuration information for the EXTI module.
  * @retval none
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == BUTTON_PIN) {

		uint32_t current_time = HAL_GetTick();

		/* Button debouncing */
		if(current_time > buttonPressTime + DEBOUNCE_THRESHOLD){
			button = !button;
			buttonPressTime = current_time;
		}
	}

	if (GPIO_Pin == GPIO_PIN_13) {
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	}
}


/**
  * @brief  User button interrupt handler
  * Override default mapping of this handler to Default_Handler
  * @param  none
  * @retval none
  */
void EXTI15_10_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(BUTTON_PIN);
}


/**
  * @brief  System Clock Configuration
  * Configuration:
  * 	Voltage Scale 		= 3
  * 	HSI clock used 		= 16 MHz
  * 	AHB, APB1, APB2 	= DIV1
  * 	APB1 				= 16MHz
  * 	APB2 				= 16MHz
  * 	Flash Latency		= 0
  * @param  None
  * @retval None
  */
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}


/**
 * @brief  Initialise ADC1 hardware
 * Configuration:
 * 		PIN					= PC0
 * 		APB2				= 16MHz
 * 		ADCCLK			 	= 8MHz
 * 		Resolution			= 8bits
 * 		Continuous			= Disabled
 * 		DMA					= Disabled
 * 		Sample time			= 3 cycles
 * 		Conversion time		= ADCCLK/(Resolution + Sample time)
 * 						= 727kHz
 * @param  None
 * @retval None
 */
void ADC1_Init(void) {

	GPIO_InitTypeDef ADC_InitStruct;

	__GPIOC_CLK_ENABLE();

	ADC_InitStruct.Pin = ADCx_CHANNEL_PIN;
	ADC_InitStruct.Mode = GPIO_MODE_ANALOG;
	ADC_InitStruct.Pull = GPIO_NOPULL;

	HAL_GPIO_Init(ADCx_CHANNEL_GPIO_PORT, &ADC_InitStruct);

	__ADC1_CLK_ENABLE();

	 ADC_ChannelConfTypeDef sConfig = {0};

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	*/
	AdcHandle.Instance = ADC1;
	AdcHandle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	AdcHandle.Init.Resolution = ADC_RESOLUTION_8B;
	AdcHandle.Init.ScanConvMode = DISABLE;
	AdcHandle.Init.ContinuousConvMode = DISABLE;
	AdcHandle.Init.DiscontinuousConvMode = DISABLE;
	AdcHandle.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	AdcHandle.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	AdcHandle.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	AdcHandle.Init.NbrOfConversion = 1;
	AdcHandle.Init.DMAContinuousRequests = DISABLE;
	AdcHandle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

	if (HAL_ADC_Init(&AdcHandle) != HAL_OK)
	{
		Error_Handler();
	}
	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	*/
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&AdcHandle, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	HAL_ADC_Start(&AdcHandle);

}

/**
 * @brief  Deinitialise ADC1 hardware
 * @param  None
 * @retval None
 */
void ADC_Deinit(void) {

	HAL_ADC_DeInit(&AdcHandle);
}


/**
 * @brief  Return current converted ADC value
 * @param  None
 * @retval uint8_t adcValue
 */
uint8_t ADC_get(void) {

    if (HAL_ADC_PollForConversion(&AdcHandle, 100) == HAL_OK) {
    	adcValue = HAL_ADC_GetValue(&AdcHandle);
    }

    /* Start new conversion */
	HAL_ADC_Start(&AdcHandle);

	return adcValue;
}



/**
 * @brief  Place current converted ADC value into
 * ADC buffer at specified position
 * @param  uint16_t pos < SAMPLES
 * @retval None
 */
void ADC_fill_buffer(uint16_t pos) {

	adcBuffer[pos] = ADC_get();
}



/**
 * @brief  Return ADC value from ADC buffer at specified position
 * @param  uint16_t pos < SAMPLES
 * @retval None
 */
uint8_t ADC_get_buffer(uint16_t pos) {

	return adcBuffer[pos];
}



/**
 * @brief  Return ADC value from ADC buffer at specified position
 * Configuration:
 * 		UART Mode		= Asynchronous mode
 * 		Word Length 	= 8 Bits
 * 		Stop Bit 		= 1
 * 		Parity 			= None
 * 		BaudRate 		= 9600
 * @param  uint16_t pos < SAMPLES
 * @retval None
 */
void BRD_debuguart_init(void) {
	UartHandle.Instance          = USARTx;

	UartHandle.Init.BaudRate     = 9600;
	UartHandle.Init.WordLength   = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits     = UART_STOPBITS_1;
	UartHandle.Init.Parity       = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode         = UART_MODE_TX_RX;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;

	GPIO_InitTypeDef GPIO_InitStruct;

    __GPIOA_CLK_ENABLE();
    __USART2_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	if(HAL_UART_Init(&UartHandle) != HAL_OK) {
	Error_Handler();
	}
}



/**
  * @brief  Transmit single char over uart
  * @param  unsigned char c
  * @retval None
  */
void BRD_debuguart_putc(unsigned char c) {

	HAL_UART_Transmit(&UartHandle, &c, 1, 0xFFFF);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpointer-sign"

/**
  * @brief  Transmit string over uart
  * @param  unsigned char *c
  * @retval None
  */
void BRD_debuguart_puts(unsigned char *c) {
	int i;

	for (i = 0; i < (int) strlen(c); i++) {
		__HAL_UART_FLUSH_DRREGISTER(&UartHandle) = (uint8_t) (*(c + i));
	}
}

#pragma GCC diagnostic pop

/**
  * @brief  Transmit len number of chars from string over uart
  * @param  unsigned char *c
  * @retval None
  */
void BRD_debuguart_putm(unsigned char *c, int len) {
	int i;
	for (i = 0; i < len; i++) {
		__HAL_UART_FLUSH_DRREGISTER(&UartHandle) = (uint8_t) (*(c + i));
	}
}


/**
  * @brief  Receive char from uart
  * @param  None
  * @retval unsigned char if successful, else null character '\0'
  */
unsigned char BRD_debuguart_getc() {

	uint8_t rx_char = '\0';

	if (HAL_UART_Receive(&UartHandle, &rx_char, 1, 0) == HAL_OK) {
		return rx_char;
	} else {
		return '\0';
	}
}


/**
  * @brief TIM2 initialization function for ADC sampling time of 10kHz
  * Configuration:
  * 	APB1			= 16MHz
  * 	Prescaler		= 99
  * 	ARR (Period)	= 15
  * 	Clock Division	= DIV1
  *		Frequency		= APB1/((Prescaler+1)(ARR+1))
  *						= 10kHz
  * @param None
  * @retval None
  */
/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
void TIM2_Init(void) {

	__TIM2_CLK_ENABLE();

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	TimHandle.Instance = TIM2;
	TimHandle.Init.Prescaler = 99;
	TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
	TimHandle.Init.Period = 15;
	TimHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

	if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK) {
		Error_Handler();
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

	if (HAL_TIM_ConfigClockSource(&TimHandle, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

	if (HAL_TIMEx_MasterConfigSynchronization(&TimHandle, &sMasterConfig) != HAL_OK){
		Error_Handler();
	}

	HAL_TIM_Base_Init(&TimHandle);

	HAL_NVIC_SetPriority(TIM2_IRQn, 10, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	HAL_TIM_Base_Start_IT(&TimHandle);
}


/**
 * @brief  Deinitialise TIM2 timer
 * @param  None
 * @retval None
 */
void TIM2_Deinit(void) {

	HAL_TIM_Base_DeInit(&TimHandle);
}


/**
  * @brief  Delay function
  * @param  uint32_t us, microseconds
  * @retval None
  */
void HAL_Delayus(uint32_t us) {
	volatile uint32_t cycles = (SystemCoreClock/1000000L)*us;
	volatile uint32_t start = DWT->CYCCNT;
	do {

	} while(DWT->CYCCNT - start < cycles);
}


/**
  * @brief  Delay function
  * @param  int counter, arbitrary counter value
  * @retval None
  */
void BRD_delay(int counter) {

	counter = counter * 16000;

	for(int i = 0; i < counter; ++i);
}


/**
  * @brief  Filters captured data - LPF: 2kHz
  * @param  None
  * @retval None
  */
void vibe_filter(void) {

	arm_fir_instance_f32 Filt;
	float32_t  *inputF32, *outputF32;

	/* Offset DC component and scale up values from ADC data */
	for (int i = 0; i < SAMPLES; i++) {
	  filtInput[i] = ((float32_t)ADC_get_buffer(i) - 127);
	}

	/* Initialize input and output buffer pointers */
	inputF32 = &filtInput[0];
	outputF32 = &filtOutput[0];

	/* Call FIR init function to initialize the instance structure. */
	arm_fir_init_f32(&Filt, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32[0], blockSize);

	/* Call the FIR process function for every blockSize samples */
	for(uint32_t i=0; i < numBlocks; i++) {
		arm_fir_f32(&Filt, inputF32 + (i * blockSize), outputF32 + (i * blockSize), blockSize);
	}
}


/**
  * @brief  Performs Fast Fourier Transform of vibration sensor data
  * @param  None
  * @retval None
  */
void vibe_fft(void) {

	arm_cfft_radix4_instance_f32 Signal;	/* ARM CFFT module */
	float32_t maxValue;				/* Max FFT value is stored here */
	uint32_t maxIndex;				/* Index in Output array where max value is */

	/* Change to complex form of ADC data */
	for (uint16_t i = 0; i < FFT_SAMPLES; i += 2) {
		/* Real part */
		fftInput[(uint16_t)i] = filtOutput[((uint16_t)i)/2];
		/* Imaginary part */
		fftInput[(uint16_t)(i + 1)] = 0;
	}

	/* Initialize the CFFT/CIFFT module */
	arm_cfft_radix4_init_f32(&Signal, FFT_SIZE, 0, 1);

	/* Process the data through the CFFT/CIFFT module */
	arm_cfft_radix4_f32(&Signal, fftInput);

	/* Process the data through the Complex Magnitude Module for calculating the magnitude at each bin */
	arm_cmplx_mag_f32(fftInput, fftOutput, FFT_SIZE);

	/* Set up package of 105 samples*/
	for (int i = 0; i < 105; i++) {
		vibrationOutput[i] = (uint8_t)(fftOutput[i]/256);
	}
	/* Remove DC - 50Hz frequency content */
	for (int i = 0; i < 5; i++) {
		vibrationOutput[i] = 0;
	}

	/* Calculates maxValue and returns corresponding value */
	arm_max_f32(fftOutput, FFT_SIZE, &maxValue, &maxIndex);
}


/* ----------------------------------------------------------------------
** The following functions are used only for debugging purposes.
** They redirect printf function to the UART so that variables and states
** can be printed out.
** --------------------------------------------------------------------- */
void debug_putc(char c) {

	BRD_debuguart_putc(c);

}

void debug_flush() {

}

unsigned char debug_getc(void) {
	uint8_t c = '\0';


	c = BRD_debuguart_getc();

	return c;
}

void debug_rxflush() {

}


PUTCHAR_PROTOTYPE
{

	BRD_debuguart_putc( (uint8_t)ch);
  return ch;
}


GETCHAR_PROTOTYPE
{
	uint8_t c = '\0';

	c = BRD_debuguart_getc();

		return c;


}


void debug_printf (const char *fmt, ...) {

	va_list args;

	  va_start (args, fmt);


	  vprintf (fmt, args);

	  va_end (args);
}
/* --------------------------------------------------------------------- */


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {

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

}
#endif /* USE_FULL_ASSERT */

