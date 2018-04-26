#include <error_handler.h>
#include "pd_phy.h"
#include "platform.h"

int main(void) {
	hw_init();

	pd_init();
	pd_rx_enable_monitoring();

	uart_puts("STM32 PD\n");
	while (1)  {
		HAL_Delay (200);

		uart_puts("V1=");
		uart_int32(adc_read(1<<2));
		uart_puts("\nV2=");
		uart_int32(adc_read(1<<4));
		uart_puts("\n10T=");
		uart_int32(adc_read_temperature());
		uart_puts("\n");
	}
}
