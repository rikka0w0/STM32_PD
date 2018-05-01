#ifndef __PD_PHY_H
#define __PD_PHY_H

#include <stdint.h>

// GPIO usages
#define PD_CC1_PIN GPIO_PIN_2
#define PD_CC2_PIN GPIO_PIN_4
#define PD_CC_GPIO GPIOA
#define PD_COMP_PIN GPIO_PIN_1	// PB1 as TIM3_CH4 during rx
#define PD_COMP_GPIO GPIOB

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
#define PD_BIT_LEN 429

/* number of edges and time window to detect CC line is not idle */
#define PD_RX_TRANSITION_COUNT  3
#define PD_RX_TRANSITION_WINDOW 20 /* between 12us and 20us */
/* Timeout for message receive in microseconds */
#define USB_PD_RX_TMOUT_US 1800
#define PD_RX_THRESHOLD 30	// @ 12 MHz Timer rate

enum pd_rx_errors {
	PD_RX_SOPPP = 3,
	PD_RX_SOPP = 2,
	PD_RX_SOP = 1,
	PD_RX_SUCCESS = 0,
	PD_RX_ERR_INVAL = -1,           /* Invalid packet */
	PD_RX_ERR_HARD_RESET = -2,      /* Got a Hard-Reset packet */
	PD_RX_ERR_CRC = -3,             /* CRC mismatch */
	PD_RX_ERR_ID = -4,              /* Invalid ID number */
	PD_RX_ERR_UNSUPPORTED_SOP = -5, /* Unsupported SOP */
	PD_RX_ERR_CABLE_RESET = -6,      /* Got a Cable-Reset packet */
	PD_RX_ERR_TIMEOUT = -7
};

enum pd_rx_special_4b5b {
	TABLE_5b4b_ERR = 0x90,
	TABLE_5b4b_SYNC1 = 0xA0,
	TABLE_5b4b_SYNC2 = 0xB0,
	TABLE_5b4b_SYNC3 = 0xC0,
	TABLE_5b4b_RST1 = 0xD0,
	TABLE_5b4b_RST2 = 0xE0,
	TABLE_5b4b_EOP = 0xF0
};

void pd_init(void);
void pd_select_cc(uint8_t cc);
void pd_rx_disable_monitoring(void);
void pd_rx_enable_monitoring(void);
uint32_t pd_rx_started(void);
int pd_rx_process(void);

#endif // __PD_PHY_H
