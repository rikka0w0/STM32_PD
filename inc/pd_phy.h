#ifndef __PD_PHY_H
#define __PD_PHY_H

/* number of edges and time window to detect CC line is not idle */
#define PD_RX_TRANSITION_COUNT  3
#define PD_RX_TRANSITION_WINDOW 20 /* between 12us and 20us */

void pd_init(void);
void pd_rx_disable_monitoring();
void pd_rx_enable_monitoring();

#endif // __PD_PHY_H
