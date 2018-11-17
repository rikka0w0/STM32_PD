#include "platform.h"
#include "error_handler.h"
#include "stm32f0xx_ll_adc.h"
#include "stm32f0xx_ll_usart.h"

extern __IO uint32_t uwTick;

// #define SYSTICK_PER_US 48

// Return timestamp in microsecond (us)
uint64_t timestamp_get(void) {
	uint32_t ms, systick;
	do {
		ms = uwTick;
		systick = SysTick->LOAD - SysTick->VAL;
	} while (ms != uwTick);
	//return ms * 1000 + systick / SYSTICK_PER_US;
	systick = ((systick >> 4) * 0x5556) >> 16;	// divide by 48, assume systick<65535
	return ((uint64_t)ms * 1000) + systick;
}

size_t uart_strlen(const char * str) {
	const char *eos = str;
	while (*eos++);
	return (eos - str - 1);
}

void uart_puts(const char *str) {
	while (*str) {
		while(!LL_USART_IsActiveFlag_TXE(USART1));
		LL_USART_TransmitData8(USART1, *str++);
		while(!LL_USART_IsActiveFlag_TC(USART1));
	}
}

void uart_put(const char c) {
	while(!LL_USART_IsActiveFlag_TXE(USART1));
	LL_USART_TransmitData8(USART1, c);
	while(!LL_USART_IsActiveFlag_TC(USART1));
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

uint16_t adc_read_temperature() {
	return TSENSOR_ADC2T100(adc_read(LL_ADC_CHANNEL_TEMPSENSOR));
};

uint16_t adc_read(uint32_t chan) {
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
	return LL_ADC_REG_ReadConversionData12(ADC1);
}

void crc32_init(void) {
	__HAL_RCC_CRC_CLK_ENABLE();
	CRC->CR = 0xE1; // Reverse output data, Reverse input data by word, Reset, RM0091 Page 224.
	while (CRC->CR & 1);
}

static void UART1_Init(void) {
	LL_USART_InitTypeDef USART_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_USART1_CLK_ENABLE();

	/**
	 USART1 GPIO Configuration
	 PA9     ------> USART1_TX
	 PA10     ------> USART1_RX
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF1_USART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	USART_InitStruct.BaudRate = 115200;
	USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
	USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
	USART_InitStruct.Parity = LL_USART_PARITY_NONE;
	USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
	USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
	LL_USART_Init(USART1, &USART_InitStruct);

	LL_USART_DisableIT_CTS(USART1);
	LL_USART_DisableOverrunDetect(USART1);
	LL_USART_DisableDMADeactOnRxErr(USART1);
	LL_USART_ConfigAsyncMode(USART1);

	LL_USART_Enable(USART1);
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

	// Enable GPIO clocks
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
}

static void ADC_Init(void) {
	LL_ADC_InitTypeDef ADC_InitStruct;
	LL_ADC_REG_InitTypeDef ADC_REG_InitStruct;

	__HAL_RCC_ADC1_CLK_ENABLE();

	LL_ADC_SetCommonPathInternalCh(__LL_ADC_COMMON_INSTANCE(ADC1),
			LL_ADC_PATH_INTERNAL_TEMPSENSOR);

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
	while (LL_ADC_IsCalibrationOnGoing(ADC1));

	LL_ADC_Enable(ADC1);
}

void hw_init(void) {
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	SystemClock_Config();
	UART1_Init();
	ADC_Init();
}
