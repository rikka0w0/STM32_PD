/*
 * USB Power delivery port controller
 * Copied from Glados-PD, original author: The Chromium OS Authors
 * */

#ifndef __PD_TCPCI_H
#define __PD_TCPCI_H

#include <stdint.h>
#include "tcpc.h"

void tcpc_init(void);
void tcpc_run(void);
uint32_t tcpc_i2c_read(uint8_t reg, uint8_t *payload);
void tcpc_i2c_write(uint8_t reg, uint32_t len, const uint8_t *payload);

// The following can be called by TCPM if TCPM is on the same CPU and same task
uint8_t tcpc_is_int_asserted(void);
void tcpc_alert_status_clear(uint16_t mask);	// Write 1 to alter register to clear flags
void tcpc_role_ctrl_change(uint8_t newval);		// Write role control register
void tcpc_look4forconnection(void);				// Write Look4Connection to command register

typedef struct __pd_message {
	// RECEIVE_BYTE_COUNT is calculated from rx_header
	// RX_BUF_FRAME_TYPE, RX_BUF_HEADER_BYTEx(WORD),  RX_BUF_OBJx_BYTE_x (7 DWORD)
	uint8_t frame_type;
	uint16_t header;
	uint32_t payload[7];
}pd_message;

uint8_t tcpc_get_message(pd_message* msg);
void tcpc_send_message(const pd_message* msg);

#endif // __PD_TCPCI_H
