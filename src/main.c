#include <error_handler.h>
#include "pd_phy.h"
#include "platform.h"

extern ADC_HandleTypeDef hadc;

int main(void) {
	hw_init();

	pd_init();
	pd_rx_enable_monitoring();

	uart_puts("STM32 PD\n");
	while (1)  {
		HAL_ADC_Start(&hadc);
		HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
		uint16_t adc1 = HAL_ADC_GetValue(&hadc);

		HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
		uint16_t adc2 = HAL_ADC_GetValue(&hadc);

		HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY);
		int temperature = TSENSOR_ADC2T100(HAL_ADC_GetValue(&hadc));

		HAL_ADC_Stop (&hadc);
		HAL_Delay (200);

		uart_puts("V1=");
		uart_int32(adc1);
		uart_puts("\nV2=");
		uart_int32(adc2);
		uart_puts("\n10T=");
		uart_int32(temperature);
		uart_puts("\n");
	}
}
