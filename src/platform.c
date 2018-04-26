#include "platform.h"
#include "error_handler.h"
#include "stm32f0xx_ll_adc.h"

UART_HandleTypeDef huart1;

extern __IO uint32_t uwTick;

#define SYSTICK_PER_US 48

// Return timestamp in microsecond (us)
uint64_t timestamp_get(void) {
	uint32_t ms, systick;
	do {
		ms = uwTick;
		systick = SysTick->VAL;
	} while (ms != uwTick);
	return ms * 1000 + systick / SYSTICK_PER_US;
}

size_t uart_strlen(const char * str) {
	const char *eos = str;
	while (*eos++)
		;
	return (eos - str - 1);
}

int uart_puts(const char *str) {
	size_t length = uart_strlen(str);
	return HAL_UART_Transmit(&huart1, (uint8_t *) (str), length, HAL_MAX_DELAY)
			== HAL_OK ? length : EOF;
}

int uart_put(const char c) {
	return HAL_UART_Transmit(&huart1, (uint8_t *) &c, 1, HAL_MAX_DELAY)
			== HAL_OK ? c : EOF;
}

void uart_int32(int n) {
	char s[16];
	int i, j, sign;

	if ((sign = n) < 0)
		n = -n;
	i = 0;
	do {
		s[i++] = n % 10 + '0';
	} while ((n /= 10) > 0);

	if (sign < 0)
		s[i++] = '-';

	for (j = i - 1; j >= 0; j--)
		uart_put(s[j]);
}

uint32_t adc_read_temperature() {
	return TSENSOR_ADC2T100(adc_read(LL_ADC_CHANNEL_TEMPSENSOR));
};

uint32_t adc_read(uint32_t chan) {
	// Configure the channel
	LL_ADC_REG_SetSequencerChannels(ADC1, chan);
	LL_ADC_REG_SetDMATransfer(ADC1, LL_ADC_REG_DMA_TRANSFER_NONE);

	/* Clear flags */
	LL_ADC_ClearFlag_EOSMP(ADC1);
	LL_ADC_ClearFlag_EOC(ADC1);
	LL_ADC_ClearFlag_EOS(ADC1);

	/* Start conversion */
	LL_ADC_REG_StartConversion(ADC1);

	/* Wait for end of conversion */
	while (LL_ADC_REG_IsConversionOngoing(ADC1));

	/* read converted value */
	return LL_ADC_REG_ReadConversionData32(ADC1);
}

static void UART1_Init(void) {
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	if (HAL_UART_Init(&huart1) != HAL_OK)
		Error_Handler();
}

static void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	// Initializes the CPU, AHB and APB busses clocks
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSI14;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.HSI14CalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
		Error_Handler();

	// Initializes the CPU, AHB and APB busses clocks
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
		Error_Handler();

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
		Error_Handler();

	// Configure the Systick interrupt time, SysTick = 1mS
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	// SysTick_IRQn interrupt configuration
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

static void ADC_Init(void) {
	  LL_ADC_InitTypeDef ADC_InitStruct;
	  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct;

	  __HAL_RCC_ADC1_CLK_ENABLE();

	  LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_PATH_INTERNAL_TEMPSENSOR);

	  // Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	  ADC_InitStruct.Clock = LL_ADC_CLOCK_ASYNC;
	  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
	  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
	  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
	  LL_ADC_Init(ADC1, &ADC_InitStruct);

	  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
	  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
	  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
	  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_LIMITED;
	  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
	  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);

	  LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);

	  LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_TEMPSENSOR);

	  LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_239CYCLES_5);

	  LL_ADC_DisableIT_EOC(ADC1);
	  LL_ADC_DisableIT_EOS(ADC1);

	  LL_ADC_StartCalibration(ADC1);
	  while(LL_ADC_IsCalibrationOnGoing(ADC1));

	  LL_ADC_Enable(ADC1);
}

void hw_init(void) {
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	SystemClock_Config();
	UART1_Init();
	ADC_Init();
}
