#include <error_handler.h>
#include "pd_phy.h"
#include "platform.h"

extern void pd_sink_run(void);
extern void pd_sink_enable(void);
extern void pd_test_tx(void);
int main(void) {
	hw_init();

	pd_init();
	pd_sink_enable();
	//pd_test_tx();
	uart_puts("STM32 PD\n");
	while (1)  {
		pd_sink_run();

	}
}
