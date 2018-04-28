#ifndef __PD_PHY_H
#define __PD_PHY_H

#include <stdint.h>

#define PD_CC1_PIN GPIO_PIN_2
#define PD_CC2_PIN GPIO_PIN_4
#define PD_CC_COMP GPIO_PIN_7
#define PD_CC_GPIO GPIOA

#define PD_CC_MASK	0x03
#define PD_CC_NC	0x00
#define PD_CC_1		0x01
#define PD_CC_2		0x02
#define PD_CC_UNDEF	PD_CC_MASK

/* number of edges and time window to detect CC line is not idle */
#define PD_RX_TRANSITION_COUNT  3
#define PD_RX_TRANSITION_WINDOW 20 /* between 12us and 20us */

void pd_init(void);
void pd_select_cc(uint8_t cc);
void pd_rx_disable_monitoring();
void pd_rx_enable_monitoring();

#endif // __PD_PHY_H
