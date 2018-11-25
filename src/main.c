#include <error_handler.h>
#include "pd_phy.h"
#include "platform.h"
#include "tcpci.h"

#define TCPM_STATE_DISCONNECTED 0
#define TCPM_STATE_ASSERTING_CONNECTION 1
#define TCPM_STATE_CONNECTED 2
#define TCPM_STATE_ASSERTING_DISCONNECTION 3
uint8_t tcpm_state;
#define PD_JUST_CONNECTED 0
#define PD_SRC_CAP_RECVED 1
#define PD_PWR_REQ_SENT 2
#define PD_WAIT_FOR_ACCEPT 3
#define PD_WAIT_FOR_PSRDY 4
#define PD_PS_OK 5
uint8_t tcpm_pd_state;
uint64_t last_timestamp;

void tcpm_run(void) {
	uint8_t buf[8];

	if (tcpc_is_int_asserted()) {
		tcpc_i2c_read(TCPC_REG_ALERT, buf);
		if (buf[0] & TCPC_REG_ALERT_CC_STATUS) {	// CC status changed
			tcpc_i2c_read(TCPC_REG_CC_STATUS, buf);

			if (tcpm_state == TCPM_STATE_DISCONNECTED) {
				if (TCPC_REG_CC_STATUS_TERM(buf[0])) {
					// Presenting Rd

					tcpm_state = TCPM_STATE_ASSERTING_CONNECTION;
					last_timestamp = timestamp_get();
				} else {
					while(1); // WTF??
				}
			} else if (tcpm_state == TCPM_STATE_ASSERTING_CONNECTION) {
				tcpm_state = TCPM_STATE_DISCONNECTED;
				tcpm_pd_state = PD_JUST_CONNECTED;
				tcpc_look4forconnection();
			} else if (tcpm_state == TCPM_STATE_CONNECTED) {
				tcpm_state = TCPM_STATE_ASSERTING_DISCONNECTION;
			}

			// Clear CC Status Flag
			buf[0] = TCPC_REG_ALERT_CC_STATUS;
			tcpc_i2c_write(TCPC_REG_ALERT, 1, buf);
		} else if (buf[0] & TCPC_REG_ALERT_RX_STATUS) {
			if (tcpm_state == TCPM_STATE_CONNECTED) {
				pd_message msg;
				uint8_t len = tcpc_get_message(&msg);


				uart_puts("packet received @");
				uart_int32(timestamp_get());
				for (uint8_t i=1; i<sizeof(pd_message); i++) {
					uart_put(' ');
					uart_hex(((uint8_t*)&msg)[i]);
				}
				uart_put('\n');

				// Clear CC Status Flag
				buf[0] = TCPC_REG_ALERT_RX_STATUS;
				tcpc_i2c_write(TCPC_REG_ALERT, 1, buf);

				if (PD_HEADER_CNT(msg.header)>0 && PD_HEADER_TYPE(msg.header) == PD_DATA_SOURCE_CAP) {
					tcpm_pd_state = PD_SRC_CAP_RECVED;
					last_timestamp = timestamp_get();
				} else if (tcpm_pd_state == PD_WAIT_FOR_ACCEPT) {
					if (PD_HEADER_CNT(msg.header)==0 && PD_HEADER_TYPE(msg.header) == PD_CTRL_ACCEPT)
						tcpm_pd_state = PD_WAIT_FOR_PSRDY;
				} else if (tcpm_pd_state == PD_WAIT_FOR_PSRDY) {
					if (PD_HEADER_CNT(msg.header)==0 && PD_HEADER_TYPE(msg.header) == PD_CTRL_PS_RDY) {
						tcpm_pd_state = PD_PS_OK;
						uart_puts("PS ready!\n");
					}
				}
			}
		} else if (buf[0] & TCPC_REG_ALERT_RX_HARD_RST) {
			if (tcpm_state == TCPM_STATE_CONNECTED) {
				uart_puts("HARD_RESET received @ ");
				uart_int32(timestamp_get());
				uart_put('\n');

				// Clear CC Status Flag
				buf[0] = TCPC_REG_ALERT_RX_HARD_RST;
				tcpc_i2c_write(TCPC_REG_ALERT, 1, buf);
			}
		} else if (buf[0] & TCPC_REG_ALERT_TX_SUCCESS) {
			// Clear CC Status Flag
			buf[0] = TCPC_REG_ALERT_TX_SUCCESS;
			tcpc_i2c_write(TCPC_REG_ALERT, 1, buf);

			if (tcpm_state == TCPM_STATE_CONNECTED && tcpm_pd_state == PD_PWR_REQ_SENT) {
				tcpm_pd_state = PD_WAIT_FOR_ACCEPT;
			}
		}
	}

	if (tcpm_state == TCPM_STATE_ASSERTING_CONNECTION) {
		if (timestamp_get() > last_timestamp + 150000) { // 150ms debouncing
			tcpm_state = TCPM_STATE_CONNECTED;

			tcpc_i2c_read(TCPC_REG_CC_STATUS, buf);

			// Assume we always have Rd
			if (TCPC_REG_CC_STATUS_CC2(buf[0]) > 1) {	// 1.5A or 3.0A
				buf[0] = 1;	// Plug Orientation = 1, monitor the CC2 pin for BMC
				uart_puts("TCPM_STATE_CONNECTED BMC=CC2\n");
			} else if (TCPC_REG_CC_STATUS_CC1(buf[0]) > 1){
				buf[0] = 0; // Plug Orientation = 0, monitor the CC1 pin for BMC
				uart_puts("TCPM_STATE_CONNECTED BMC=CC1\n");
			}
			tcpc_i2c_write(TCPC_REG_TCPC_CTRL, 1, buf);	// Apply Rp/Rd according to CC_Status.ConnectionResult

			buf[0] = TCPC_REG_RX_DETECT_HRST | TCPC_REG_RX_DETECT_SOP;
			tcpc_i2c_write(TCPC_REG_RX_DETECT, 1, buf);
		}
	} else if (tcpm_state == TCPM_STATE_ASSERTING_DISCONNECTION) {
		if (timestamp_get() > last_timestamp + 50000) {	// 50ms debouncing
			tcpm_state = TCPM_STATE_DISCONNECTED;
			uart_puts("TCPM_STATE_DISCONNECTED\n");
		}
	} else if (tcpm_state == TCPM_STATE_CONNECTED) {
		if (tcpm_pd_state == PD_SRC_CAP_RECVED && timestamp_get() > last_timestamp + 1000) {
			tcpm_pd_state = PD_PWR_REQ_SENT;

			pd_message msg;
			msg.frame_type = 0;
			msg.header = PD_HEADER(
					PD_DATA_REQUEST,
					0,
					0,
					0,	//ID
					1,
					1,
					0);	// 0x1042
			msg.payload[0] = 0x430320C8;
			tcpc_send_message(&msg);
		}
	}
}

int main(void) {
	hw_init();

	tcpc_init();

	uint8_t cmd[8];

	// Clear POWER_STATUS Flag
	cmd[0] = TCPC_REG_ALERT_POWER_STATUS;
	tcpc_i2c_write(TCPC_REG_ALERT, 1, cmd);
	cmd[2] = tcpc_is_int_asserted();
	tcpm_state = TCPM_STATE_DISCONNECTED;

	cmd[0] = TCPC_REG_ROLE_CTRL_SET(0,0,TYPEC_CC_RD,TYPEC_CC_RD);
	tcpc_i2c_write(TCPC_REG_ROLE_CTRL, 1, cmd);
	tcpc_look4forconnection();

	//uart_puts("STM32 PD\n");
	while (1)  {
		tcpc_run();
		tcpm_run();
	}
}
