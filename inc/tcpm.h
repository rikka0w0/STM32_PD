#ifndef __PD_TCPM_H
#define __PD_TCPM_H

#include <stdint.h>
#include "tcpc.h"

// Available flags
#define CONFIG_USB_PD_VBUS_DETECT_TCPC
#define CONFIG_USB_PD_DISCHARGE_TCPC
#define CONFIG_USB_PD_DUAL_ROLE_AUTO_TOGGLE

void tcpm_get_cc(int port, int* cc1, int* cc2);

#ifdef CONFIG_USB_PD_VBUS_DETECT_TCPC
int tcpm_get_vbus_level(int port);
#endif

void tcpm_select_rp_value(int port, int rp);
void tcpm_set_cc(int port, int pull);
void tcpm_set_polarity(int port, int polarity);
void tcpm_set_vconn(int port, int enable);
void tcpm_set_msg_header(int port, int power_role, int data_role);
void tcpm_set_rx_enable(int port, int enable);
void tcpm_get_message(int port, uint32_t *payload, int *head);
void tcpm_transmit(int port, enum tcpm_transmit_type type, uint16_t header, const uint32_t *data);

#ifdef CONFIG_USB_PD_DISCHARGE_TCPC
void tcpc_discharge_vbus(int port, int enable);
#endif

#ifdef CONFIG_USB_PD_DUAL_ROLE_AUTO_TOGGLE
void tcpm_set_drp_toggle(int port, int enable);
#endif

#endif // #ifndef __PD_TCPM_H
