#ifndef __PD_PHY_H
#define __PD_PHY_H

#include <stdint.h>

// CC GPIO mapping
#define PD_CC1_PIN GPIO_PIN_2
#define PD_CC2_PIN GPIO_PIN_4
#define PD_CC_GPIO GPIOA

// CC status
#define PD_CC_MASK	0x03
#define PD_CC_NC	0x00
#define PD_CC_1		0x01
#define PD_CC_2		0x02
#define PD_CC_UNDEF	PD_CC_MASK

/*
 * Maximum size of a Power Delivery packet (in bits on the wire) :
 *    16-bit header + 0..7 32-bit data objects  (+ 4b5b encoding)
 * 64-bit preamble + SOP (4x 5b) + message in 4b5b + 32-bit CRC + EOP (1x 5b)
 * = 64 + 4*5 + 16 * 5/4 + 7 * 32 * 5/4 + 32 * 5/4 + 5
 */
#define PD_BIT_LEN 428
#define PD_MAX_RAW_SIZE (PD_BIT_LEN*2)

/* number of edges and time window to detect CC line is not idle */
#define PD_RX_TRANSITION_COUNT  3
#define PD_RX_TRANSITION_WINDOW 20 /* between 12us and 20us */
/* Timeout for message receive in microseconds */
#define USB_PD_RX_TMOUT_US 1800
#define PD_RX_THRESHOLD 30	// @ 12 MHz Timer rate

enum pd_rx_sop_types {	// negative value indicates error
	PD_RX_ERR_CABLE_RESET = 6,
	PD_RX_ERR_HARD_RESET = 5,
	PD_RX_SOP_DBGPP = 4,
	PD_RX_SOP_DBGP = 3,
	PD_RX_SOPPP = 2,
	PD_RX_SOPP = 1,
	PD_RX_SOP = 0,

	PD_RX_IDLE = 0xFF,				// Idle, no packet received
	PD_RX_ERR_TIMEOUT = 0xFE,		// 0xFF, Timeout
	PD_RX_ERR_INVAL = 0xFD,           /* Invalid packet */
	PD_RX_ERR_CRC = 0xFC,             /* CRC mismatch */
	PD_RX_ERR_UNSUPPORTED_SOP = 0xFB, /* Unsupported SOP */
	PD_RX_ERR_OVERRUN = 0xFA		// A message arrived before the TCPC copies the old message from phy
};

enum pd_rx_special_4b5b {
	TABLE_5b4b_SYNC1 = 16,
	TABLE_5b4b_SYNC2 = 17,
	TABLE_5b4b_RST1 = 18,
	TABLE_5b4b_RST2 = 19,
	TABLE_5b4b_EOP = 20,
	TABLE_5b4b_SYNC3 = 21,
	TABLE_5b4b_ERR = 22
};

void pd_init(void);
void pd_select_cc(uint8_t cc);

void pd_rx_disable_monitoring(void);
void pd_rx_enable_monitoring(void);
uint32_t pd_rx_started(void);

uint8_t pd_phy_get_rx_type(void);
void pd_phy_clear_rx_type(void);
uint16_t pd_phy_get_rx_msg(uint8_t* payload);
uint16_t tcpc_phy_get_goodcrc_header(uint8_t rx_result, uint8_t id);	// Used by PHY, Implemented in TCPC

char pd_tx(void);
void pd_prepare_message(uint8_t sop_type, uint8_t cnt, const uint8_t* data);

// CC Rp/Rd & Vconn control
void pd_cc_rprp_init(void);
uint8_t pd_cc_read_status(uint8_t cc, uint8_t resistor, uint8_t rp);
void pd_cc_set(uint8_t role_ctrl_regval);
#endif // __PD_PHY_H
