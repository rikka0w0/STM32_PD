#include <error_handler.h>
#include "pd_phy.h"
#include "platform.h"
#include "tcpci.h"

int main(void) {
	hw_init();

	tcpc_init();

	uint8_t cmd[2] = {TCPC_REG_ROLE_CTRL, TCPC_REG_ROLE_CTRL_SET(0,0,TYPEC_CC_RD,TYPEC_CC_RD)};
	tcpc_i2c_process(0, sizeof(cmd), cmd);

	//uart_puts("STM32 PD\n");
	while (1)  {
		tcpc_run();
	}
}
