/*
 * USB Power delivery port controller
 * Copied from Glados-PD, original author: The Chromium OS Authors
 * */

#ifndef __PD_TCPM_TCPCI_H
#define __PD_TCPM_TCPCI_H

#include "pd.h"
#include <stdint.h>

void tcpc_init(void);
void tcpc_run(void);
uint32_t tcpc_i2c_read(uint8_t reg, uint8_t *payload);
void tcpc_i2c_write(uint8_t reg, uint32_t len, const uint8_t *payload);

// The following can be called by TCPM if TCPM is on the same CPU and same task
uint8_t tcpc_is_int_asserted(void);
void tcpc_alert_status_clear(uint16_t mask);
void tcpc_look4forconnection(void);

typedef struct __pd_message {
	// RECEIVE_BYTE_COUNT is calculated from rx_header
	// RX_BUF_FRAME_TYPE, RX_BUF_HEADER_BYTEx(WORD),  RX_BUF_OBJx_BYTE_x (7 DWORD)
	uint8_t frame_type;
	uint16_t header;
	uint32_t payload[7];
}pd_message;

uint8_t tcpc_get_message(pd_message* msg);

enum tcpc_cc_voltage_status {
	TYPEC_CC_VOLT_OPEN = 0,		// SRC.Open, SNK.Open
	TYPEC_CC_VOLT_RA = 1,		// SRC.Ra
	TYPEC_CC_VOLT_RD = 2,		// SRC.Rd
	TYPEC_CC_VOLT_SNK_DEF = 5,	// SNK.Rp
	TYPEC_CC_VOLT_SNK_1_5 = 6,	// SNK.Rp
	TYPEC_CC_VOLT_SNK_3_0 = 7,	// SNK.Rp
};

enum tcpc_cc_pull {
	TYPEC_CC_RA = 0,
	TYPEC_CC_RP = 1,
	TYPEC_CC_RD = 2,
	TYPEC_CC_OPEN = 3,
};

enum tcpc_rp_value {
	TYPEC_RP_USB = 0,
	TYPEC_RP_1A5 = 1,
	TYPEC_RP_3A0 = 2,
	TYPEC_RP_RESERVED = 3,
};

enum tcpm_transmit_type {
	TCPC_TX_SOP = 0,
	TCPC_TX_SOP_PRIME = 1,
	TCPC_TX_SOP_PRIME_PRIME = 2,
	TCPC_TX_SOP_DEBUG_PRIME = 3,
	TCPC_TX_SOP_DEBUG_PRIME_PRIME = 4,
	TCPC_TX_HARD_RESET = 5,
	TCPC_TX_CABLE_RESET = 6,
	TCPC_TX_BIST_MODE_2 = 7
};


/* TCPC Registers */
#define TCPC_REG_VENDOR_ID         0x0
#define TCPC_REG_PRODUCT_ID        0x2
#define TCPC_REG_BCD_DEV           0x4
#define TCPC_REG_TC_REV            0x6
#define TCPC_REG_PD_REV            0x8
#define TCPC_REG_PD_INT_REV        0xa
#define TCPC_REG_ALERT             0x10

#define TCPC_REG_ALERT_MASK_ALL     0xfff
#define TCPC_REG_ALERT_VBUS_DISCNCT (1<<11)
#define TCPC_REG_ALERT_RX_BUF_OVF   (1<<10)
#define TCPC_REG_ALERT_FAULT        (1<<9)
#define TCPC_REG_ALERT_V_ALARM_LO   (1<<8)
#define TCPC_REG_ALERT_V_ALARM_HI   (1<<7)
#define TCPC_REG_ALERT_TX_SUCCESS   (1<<6)
#define TCPC_REG_ALERT_TX_DISCARDED (1<<5)
#define TCPC_REG_ALERT_TX_FAILED    (1<<4)
#define TCPC_REG_ALERT_RX_HARD_RST  (1<<3)
#define TCPC_REG_ALERT_RX_STATUS    (1<<2)
#define TCPC_REG_ALERT_POWER_STATUS (1<<1)
#define TCPC_REG_ALERT_CC_STATUS    (1<<0)
#define TCPC_REG_ALERT_TX_COMPLETE  (TCPC_REG_ALERT_TX_SUCCESS | \
				      TCPC_REG_ALERT_TX_DISCARDED | \
				      TCPC_REG_ALERT_TX_FAILED)

#define TCPC_REG_ALERT_MASK        0x12
#define TCPC_REG_POWER_STATUS_MASK 0x14
#define TCPC_REG_FAULT_STATUS_MASK 0x15
#define TCPC_REG_CONFIG_STD_OUTPUT 0x18

#define TCPC_REG_CONFIG_STD_OUTPUT_MUX_MASK          (3 << 2)
#define TCPC_REG_CONFIG_STD_OUTPUT_MUX_NONE          (0 << 2)
#define TCPC_REG_CONFIG_STD_OUTPUT_MUX_USB           (1 << 2)
#define TCPC_REG_CONFIG_STD_OUTPUT_MUX_DP            (2 << 2)
#define TCPC_REG_CONFIG_STD_OUTPUT_CONNECTOR_FLIPPED (1 << 0)

#define TCPC_REG_TCPC_CTRL         0x19
#define TCPC_REG_TCPC_CTRL_SET(polarity) (polarity)
#define TCPC_REG_TCPC_CTRL_POLARITY(reg) ((reg) & 0x1)

/* ROLE_CONTROL Register */
#define TCPC_REG_ROLE_CTRL         0x1a
#define TCPC_REG_ROLE_CTRL_SET(drp, rp, cc1, cc2) \
		((drp) << 6 | (rp) << 4 | (cc2) << 2 | (cc1))
#define TCPC_REG_ROLE_CTRL_DRP_MASK 0x40
#define TCPC_REG_ROLE_CTRL_DRP(reg) (((reg) & TCPC_REG_ROLE_CTRL_DRP_MASK) >> 6)
#define TCPC_REG_ROLE_CTRL_CCXRP(reg) ((reg) & 0x3F)
#define TCPC_REG_ROLE_CTRL_RP_MASK  0x30
#define TCPC_REG_ROLE_CTRL_RP(reg)  (((reg) & TCPC_REG_ROLE_CTRL_RP_MASK) >> 4)
#define TCPC_REG_ROLE_CTRL_CC2(reg) (((reg) & 0xc) >> 2)
#define TCPC_REG_ROLE_CTRL_CC1(reg) ((reg) & 0x3)
/* End of ROLE_CONTROL Register */

#define TCPC_REG_FAULT_CTRL        0x1b
#define TCPC_REG_POWER_CTRL        0x1c
#define TCPC_REG_POWER_CTRL_FORCE_DISCHARGE  (1 << 2)
#define TCPC_REG_POWER_CTRL_SET(vconn) (vconn)
#define TCPC_REG_POWER_CTRL_VCONN(reg)    ((reg) & 0x1)

#define TCPC_REG_CC_STATUS         0x1d
#define TCPC_REG_CC_STATUS_LOOK4CONNECTION_MASK 0x20
#define TCPC_REG_CC_STATUS_SET(term, cc1, cc2) \
		((term) << 4 | ((cc2) & 0x3) << 2 | ((cc1) & 0x3))
#define TCPC_REG_CC_STATUS_TERM(reg) (((reg) & 0x10) >> 4)
#define TCPC_REG_CC_STATUS_CC2(reg)  (((reg) & 0xc) >> 2)
#define TCPC_REG_CC_STATUS_CC1(reg)  ((reg) & 0x3)

#define TCPC_REG_POWER_STATUS      0x1e
#define TCPC_REG_POWER_STATUS_MASK_ALL  0xff
#define TCPC_REG_POWER_STATUS_VBUS_PRES (1<<2)
#define TCPC_REG_POWER_STATUS_VBUS_DET  (1<<3)
#define TCPC_REG_POWER_STATUS_UNINIT    (1<<6)
#define TCPC_REG_FAULT_STATUS      0x1f

#define TCPC_REG_COMMAND           0x23
#define TCPC_REG_COMMAND_LOOK4CONNECTION	0x99
#define TCPC_REG_COMMAND_I2CIDLE		0xFF

#define TCPC_REG_DEV_CAP_1         0x24
#define TCPC_REG_DEV_CAP_2         0x26
#define TCPC_REG_STD_INPUT_CAP     0x28
#define TCPC_REG_STD_OUTPUT_CAP    0x29

#define TCPC_REG_MSG_HDR_INFO      0x2e
#define TCPC_REG_MSG_HDR_INFO_SET(drole, prole, rev) \
		((drole) << 3 | (rev << 1) | (prole))
#define TCPC_REG_MSG_HDR_INFO_DROLE(reg) (((reg) & 0x8) >> 3)
#define TCPC_REG_MSG_HDR_INFO_PROLE(reg) ((reg) & 0x1)
#define TCPC_REG_MSG_HDR_INFO_REV(reg) (((reg) & 0x6) >> 1)

// RECEIVE_DETECT register
#define TCPC_REG_RX_DETECT         0x2F
#define TCPC_REG_RX_DETECT_CRST 0x40	// Cable Reset
#define TCPC_REG_RX_DETECT_HRST 0x20	// Hard Reset
#define TCPC_REG_RX_DETECT_SOP_DBGPP 0x10
#define TCPC_REG_RX_DETECT_SOP_DBGP 0x08
#define TCPC_REG_RX_DETECT_SOPPP 0x04
#define TCPC_REG_RX_DETECT_SOPP 0x02
#define TCPC_REG_RX_DETECT_SOP 0x01
#define TCPC_REG_RX_ENABLED(regval) (regval&0x7F)
#define TCPC_REG_RX_DETECT_SOP_HRST_MASK (TCPC_REG_RX_DETECT_HRST|TCPC_REG_RX_DETECT_CRST)

#define TCPC_REG_RX_BYTE_CNT       0x30
#define TCPC_REG_RX_BUF_FRAME_TYPE 0x31

#define TCPC_REG_RX_HDR            0x32
#define TCPC_REG_RX_DATA           0x34 /* through 0x4f */

#define TCPC_REG_TRANSMIT          0x50
#define TCPC_REG_TRANSMIT_SET(type) \
		(PD_RETRY_COUNT << 4 | (type))
#define TCPC_REG_TRANSMIT_RETRY(reg) (((reg) & 0x30) >> 4)
#define TCPC_REG_TRANSMIT_TYPE(reg)  ((reg) & 0x7)

#define TCPC_REG_TX_BYTE_CNT       0x51
#define TCPC_REG_TX_HDR            0x52
#define TCPC_REG_TX_DATA           0x54 /* through 0x6f */

#define TCPC_REG_VBUS_VOLTAGE                0x70
#define TCPC_REG_VBUS_SINK_DISCONNECT_THRESH 0x72
#define TCPC_REG_VBUS_STOP_DISCHARGE_THRESH  0x74
#define TCPC_REG_VBUS_VOLTAGE_ALARM_HI_CFG   0x76
#define TCPC_REG_VBUS_VOLTAGE_ALARM_LO_CFG   0x78

#endif
