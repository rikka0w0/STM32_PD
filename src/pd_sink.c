#include "platform.h"
#include "pd_phy.h"

// UFP CC line threshold value (With RD)
//	ICapaticy 	CC_Min		CC_Max
//	Default		0.25V		0.61V
//	1.5A		0.70V		1.16V
//	3.0A		1.31V		2.04V
#define  PD_RDV_DEFAULT_MIN	310
#define  PD_RDV_DEFAULT_MAX	757
#define  PD_RDV_1A5_MIN		868
#define  PD_RDV_1A5_MAX		1440
#define  PD_RDV_3A0_MIN		1625
#define  PD_RDV_3A0_MAX		2532

#define PD_PORTSTATE_MASK			0x30
#define PD_PORTSTATE_DISABLED		0x00
#define PD_PORTSTATE_DISCONNECTED	0x10
#define PD_PORTSTATE_ASSERTING		0x20
#define PD_PORTSTATE_CONNECTED		0x30

#define PD_ICAP_MASK 		0x0C
#define PD_ICAP_NC			0x00
#define PD_ICAP_DEF			0x04
#define PD_ICAP_1A5			0x08
#define PD_ICAP_3A0			0x0C

volatile uint32_t pd_port_status;

uint8_t pd_get_last_cc() {
	return pd_port_status & PD_CC_MASK;
}

// Detect CC line and return advertised current capacity, Return PD_ICAP_XXX
uint8_t pd_detect_cc(uint8_t cc) {
	uint16_t adc_val;
	if (cc == PD_CC_1) {
		adc_val = adc_read(PD_CC1_PIN);
	} else if (cc == PD_CC_2) {
		adc_val = adc_read(PD_CC2_PIN);
	} else {
		return PD_ICAP_NC;
	}

	if((adc_val >= PD_RDV_3A0_MIN) && (adc_val <= PD_RDV_3A0_MAX)) {
		return PD_ICAP_3A0;
	} else if((adc_val >= PD_RDV_1A5_MIN) && (adc_val <= PD_RDV_1A5_MAX)) {
		return PD_ICAP_1A5;
	} else if((adc_val >= PD_RDV_DEFAULT_MIN) && (adc_val <= PD_RDV_DEFAULT_MAX)) {
		return PD_ICAP_DEF;
	} else {
		return PD_ICAP_NC;
	}
}

void pd_sink_enable(void) {
	if ((pd_port_status & PD_PORTSTATE_MASK) == PD_PORTSTATE_DISABLED) {
		pd_port_status &=~ PD_PORTSTATE_MASK;
		pd_port_status |= PD_PORTSTATE_DISCONNECTED;
	}
}

void pd_sink_disable(void) {
	pd_port_status &=~ PD_PORTSTATE_MASK;
	pd_port_status |= PD_PORTSTATE_DISABLED;
}

__weak void pd_cc_connected(void) {
	if ((pd_port_status & PD_CC_MASK) == PD_CC_1) {
		pd_select_cc(PD_CC_1);
		pd_rx_enable_monitoring();
		uart_puts("CC1 connected\n");
	} else if ((pd_port_status & PD_CC_MASK) == PD_CC_2) {
		pd_select_cc(PD_CC_2);
		pd_rx_enable_monitoring();
		uart_puts("CC2 connected\n");
	}
}

__weak void pd_cc_disconnected(void) {
	pd_rx_disable_monitoring();
	pd_select_cc(PD_CC_NC);
	uart_puts("CC disconnected\n");
}

// This delay quits as soon as rx starts
void pd_delay(uint32_t wait) {
  uint32_t tickstart = HAL_GetTick();

  /* Add a period to guarantee minimum wait */
  if (wait < HAL_MAX_DELAY)
     wait++;

  while((HAL_GetTick() - tickstart) < wait && !pd_rx_started());
}

void pd_sink_run(void) {
	static uint32_t last_tick;

	switch (pd_port_status & PD_PORTSTATE_MASK) {
	case PD_PORTSTATE_DISCONNECTED:
		if (pd_detect_cc(PD_CC_1) != PD_ICAP_NC) {
			pd_port_status &=~ (PD_CC_MASK|PD_PORTSTATE_MASK);
			pd_port_status |= PD_CC_1 | PD_PORTSTATE_ASSERTING;
			last_tick = HAL_GetTick();
		} else if (pd_detect_cc(PD_CC_2) != PD_ICAP_NC) {
			pd_port_status &=~ (PD_CC_MASK|PD_PORTSTATE_MASK);
			pd_port_status |= PD_CC_2 | PD_PORTSTATE_ASSERTING;
			last_tick = HAL_GetTick();
		}
		break;
	case PD_PORTSTATE_ASSERTING:
		if (HAL_GetTick() > last_tick + 10) {
			pd_port_status &=~ PD_PORTSTATE_MASK;
			if ((pd_port_status & PD_CC_MASK) == PD_CC_1) {
				if (pd_detect_cc(PD_CC_1) == PD_ICAP_NC) {
					pd_port_status |= PD_PORTSTATE_DISCONNECTED;
				} else {
					pd_port_status |= PD_PORTSTATE_CONNECTED;
					pd_cc_connected();
				}
			} else if ((pd_port_status & PD_CC_MASK) == PD_CC_2) {
				if (pd_detect_cc(PD_CC_2) == PD_ICAP_NC) {
					pd_port_status |= PD_PORTSTATE_DISCONNECTED;
				} else {
					pd_port_status |= PD_PORTSTATE_CONNECTED;
					pd_cc_connected();
				}
			} else {
				pd_port_status |= PD_PORTSTATE_DISCONNECTED;
			}
		}
		break;
	case PD_PORTSTATE_CONNECTED:
		if (pd_rx_started()) {
			//pd_rx_process();
		} else {
			if ((pd_port_status & PD_CC_MASK) == PD_CC_1) {
				if (pd_detect_cc(PD_CC_1) == PD_ICAP_NC) {
					pd_delay(10);
					if (pd_rx_started())
						return;
					if (pd_detect_cc(PD_CC_1) == PD_ICAP_NC) {
						pd_port_status &=~ (PD_CC_MASK|PD_PORTSTATE_MASK);
						pd_port_status |= PD_CC_NC | PD_PORTSTATE_DISCONNECTED;
						pd_cc_disconnected();
					}
				}
			} else if ((pd_port_status & PD_CC_MASK) == PD_CC_2) {
				if (pd_detect_cc(PD_CC_2) == PD_ICAP_NC) {
					pd_delay(10);
					if (pd_rx_started())
						return;
					if (pd_detect_cc(PD_CC_2) == PD_ICAP_NC) {
						pd_port_status &=~ (PD_CC_MASK|PD_PORTSTATE_MASK);
						pd_port_status |= PD_CC_NC | PD_PORTSTATE_DISCONNECTED;
						pd_cc_disconnected();
					}
				}
			}
		}

//		HAL_Delay (1000);
//		uart_puts("V1=");
//		uart_int32(adc_read(1<<2));
//		uart_puts("\nV2=");
//		uart_int32(adc_read(1<<4));
//		uart_puts("\n10T=");
//		uart_int32(adc_read_temperature());
//		uart_puts("\n");
		break;
	default:
		break;
	}
}
