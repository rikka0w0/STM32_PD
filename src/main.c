#include <error_handler.h>
#include "platform.h"
#include "tcpci.h"
#include "pd.h"

extern void pd_protocol_run(); extern void pd_protocol_init();
int main(void) {
	hw_init();

	tcpc_init();
	pd_protocol_init();

	//uart_puts("STM32 PD\n");
	while (1)  {
		tcpc_run();
		pd_protocol_run();
	}
}
