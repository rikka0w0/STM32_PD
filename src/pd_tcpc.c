#include "platform.h"
#include "pd_phy.h"
#include "tcpci.h"

#define TCPC_TIMER_TOGGLE_SNK 60000
#define TCPC_TIMER_TOGGLE_SRC 60000

// internal_flags
#define TCPC_FLAG_LOOKING4CON (1<<0)
#define TCPC_FLAG_DRP_TOGGLE_AS_SNK (1<<1)
#define TCPC_FLAG_DRP_TOGGLE_AS_SRC (1<<2)
#define TCPC_FLAG_CON_RESULT (1<<3)
#define TCPC_FLAG_DEBOUNCING (1<<4)
#define TCPC_FLAG_INT_ASSERTED (1<<5)

#define RX_BUFFER_SIZE 1

static struct pd_port_controller {
	/* current port power role (SOURCE or SINK) */
	uint8_t power_role;
	/* current port data role (DFP or UFP) */
	uint8_t data_role;

	/* Port polarity : 0 => CC1 is CC line, 1 => CC2 is CC line */
	uint8_t cc_tcpc_ctrl;	// TCPC_Control bit0
	uint8_t cc_power_ctrl;
	uint8_t cc_role_ctrl;
	/* CC status */
	uint8_t cc_status[2];	// tcpc_cc_voltage_status

	/* TCPC alert status */
	uint16_t alert;
	uint16_t alert_mask;
	/* RX enabled */
	uint8_t reg_recv_detect;
	/* Power status */
	uint8_t power_status;
	uint8_t power_status_mask;

	/* Last received */
	int rx_head[RX_BUFFER_SIZE+1];
	uint32_t rx_payload[RX_BUFFER_SIZE+1][7];
	int rx_buf_head, rx_buf_tail;

	/* Next transmit */
	enum tcpm_transmit_type tx_type;
	uint16_t tx_head;
	uint32_t tx_payload[7];
	const uint32_t *tx_data;

	/* Internal Flags */
	uint32_t internal_flags;
	uint64_t drp_last_toggle_timestamp;
	uint64_t cc_debouncing_timestamp;
	uint64_t cc_last_sampled_timestamp;
} pd;

static void alert(uint16_t mask)
{
	/* Always update the Alert status register */
	pd.alert |= mask;
	/*
	 * Only send interrupt to TCPM if corresponding
	 * bit in the alert_enable register is set.
	 */
	if (pd.alert_mask & mask) {
		__asm__ __volatile__("cpsid i");
		pd.internal_flags |= TCPC_FLAG_INT_ASSERTED;
		__asm__ __volatile__("cpsie i");
	}
}

static void tcpc_alert_status_clear(uint16_t mask)
{
	/*
	 * If the RX status alert is attempting to be cleared, then increment
	 * rx buffer tail pointer. if the RX buffer is not empty, then keep
	 * the RX status alert active.
	 */
//	if (mask & TCPC_REG_ALERT_RX_STATUS) {
//		if (!rx_buf_is_empty(port)) {
//			rx_buf_increment(port, &pd[port].rx_buf_tail);
//			if (!rx_buf_is_empty(port))
//				/* buffer is not empty, keep alert active */
//				mask &= ~TCPC_REG_ALERT_RX_STATUS;
//		}
//	}

	/* clear only the bits specified by the TCPM */
	pd.alert &= ~mask;

	/* Set Alert# inactive if all alert bits clear */
	if (!pd.alert) {
		__asm__ __volatile__("cpsid i");
		pd.internal_flags &=~ TCPC_FLAG_INT_ASSERTED;
		__asm__ __volatile__("cpsie i");
	}
		//tcpc_alert_clear(port);
}

void tcpc_init(void)
{
	pd.power_status |= TCPC_REG_POWER_STATUS_UNINIT | TCPC_REG_POWER_STATUS_VBUS_DET;

	// Initialize CC pull-up/pull-down and Vconn controller
	pd_cc_rprp_init();

	// Initialize PD BMC transceiver
	pd_init();

	pd.cc_role_ctrl = TCPC_REG_ROLE_CTRL_SET(0, TYPEC_RP_USB, TYPEC_CC_RD, TYPEC_CC_RD);
	pd.reg_recv_detect = 0;

	/* set default alert and power mask register values */
	pd.alert_mask = TCPC_REG_ALERT_MASK_ALL;
	pd.power_status_mask = TCPC_REG_POWER_STATUS_MASK_ALL;

	// Init done!
	pd.power_status &=~ TCPC_REG_POWER_STATUS_UNINIT;
	alert(TCPC_REG_ALERT_POWER_STATUS);
}

uint8_t tcpc_is_int_asserted() {
	return (pd.internal_flags&TCPC_FLAG_INT_ASSERTED) ? 1 : 0;
}

void tcpc_look4forconnection(void)
{
	uint8_t cc1 = TCPC_REG_ROLE_CTRL_CC1(pd.cc_role_ctrl);
	uint8_t cc2 = TCPC_REG_ROLE_CTRL_CC2(pd.cc_role_ctrl);

	if (cc1 != cc2)	// If cc1 and cc2 are not both Rp/Rd at the same time, then return
		return;

	if (cc2 == TYPEC_CC_RP || cc2 == TYPEC_CC_RD)
		pd.internal_flags |= TCPC_FLAG_LOOKING4CON;

	if (pd.cc_role_ctrl & TCPC_REG_ROLE_CTRL_DRP_MASK) {
		// DRP auto-toggle is enabled
		if (cc2 == TYPEC_CC_RP) {
			pd.internal_flags |= TCPC_FLAG_DRP_TOGGLE_AS_SRC;
			pd.drp_last_toggle_timestamp = timestamp_get();
		} else if (cc2 == TYPEC_CC_RD) {
			pd.internal_flags |= TCPC_FLAG_DRP_TOGGLE_AS_SNK;
			pd.drp_last_toggle_timestamp = timestamp_get();
		}
	}
}

static inline void tcpc_role_ctrl_change(uint8_t newval)
{
	pd.internal_flags &=~ (TCPC_FLAG_LOOKING4CON | TCPC_FLAG_DRP_TOGGLE_AS_SNK | TCPC_FLAG_DRP_TOGGLE_AS_SRC);

	// Update DRP bit
	pd.cc_role_ctrl &= ~ TCPC_REG_ROLE_CTRL_DRP_MASK;
	pd.cc_role_ctrl |= newval & TCPC_REG_ROLE_CTRL_DRP_MASK;

	if (TCPC_REG_ROLE_CTRL_CCXRP(pd.cc_role_ctrl) == TCPC_REG_ROLE_CTRL_CCXRP(newval))
		return;	// Nothing changed, do not change Rp/Rd

	// Rp/Rd changed

	pd.cc_role_ctrl = newval;	// DRP bit has been updated already
	pd_cc_set(TCPC_REG_ROLE_CTRL_CCXRP(pd.cc_role_ctrl));

	/*
	 * Before CC pull can be changed and the task can read the new
	 * status, we should set the CC status to open, in case TCPM
	 * asks before it is known for sure.
	 */
	pd.cc_status[0] = TYPEC_CC_VOLT_OPEN;
	pd.cc_status[1] = TYPEC_CC_VOLT_OPEN;

	pd.cc_last_sampled_timestamp = timestamp_get();	// Postponed the next sampling of CC line, debouncing
}

void tcpc_i2c_write(uint8_t reg, uint32_t len, const uint8_t *payload)
{
	uint16_t alert;

	if (len < 1)
		return;

	/* If we are not yet initialized, ignore any write command */
	if (pd.power_status & TCPC_REG_POWER_STATUS_UNINIT)
		return;

	switch (reg) {

	// Alert & Masks
	case TCPC_REG_ALERT:	// Write bit1 to clear corresponding bit
		alert = payload[0];
		alert |= (payload[1] << 8);
		/* clear alert bits specified by the TCPM */
		tcpc_alert_status_clear(alert);
		break;
	case TCPC_REG_ALERT_MASK:
		pd.alert_mask = payload[0];
		pd.alert_mask |= (payload[1] << 8);
		break;

	// Control registers
	case TCPC_REG_TCPC_CTRL:
		pd.cc_tcpc_ctrl = payload[0];
		pd_select_cc(TCPC_REG_TCPC_CTRL_POLARITY(pd.cc_tcpc_ctrl) ? 2 : 1);
		break;
	case TCPC_REG_ROLE_CTRL:
		tcpc_role_ctrl_change(payload[0]);
		break;
	case TCPC_REG_POWER_CTRL:
		//tcpc_set_vconn(TCPC_REG_POWER_CTRL_VCONN(payload[1]));
		break;

	// Command register, write only
	case TCPC_REG_COMMAND:
		if (payload[0] == TCPC_REG_COMMAND_LOOK4CONNECTION) tcpc_look4forconnection();
		break;


	case TCPC_REG_MSG_HDR_INFO:
		//tcpc_set_msg_header(TCPC_REG_MSG_HDR_INFO_PROLE(payload[1]),
		//		    TCPC_REG_MSG_HDR_INFO_DROLE(payload[1]));
		break;

	case TCPC_REG_RX_DETECT:
		pd.reg_recv_detect = payload[0];

		if (!TCPC_REG_RX_ENABLED(pd.reg_recv_detect))
			pd_rx_disable_monitoring();

		break;
	case TCPC_REG_POWER_STATUS_MASK:
		//tcpc_set_power_status_mask(payload[1]);
		break;
	case TCPC_REG_TX_HDR:
		pd.tx_head = (payload[2] << 8) | payload[1];
		break;
	case TCPC_REG_TX_DATA:
		//memcpy(pd.tx_payload, &payload[1], len - 1);
		break;
	case TCPC_REG_TRANSMIT:
		//tcpc_transmit(TCPC_REG_TRANSMIT_TYPE(payload[1]),
		//	      pd.tx_head, pd.tx_payload);
		break;
	}
}

uint32_t tcpc_i2c_read(uint8_t reg, uint8_t *payload)
{
	switch (reg) {

	// Alert & Masks
	case TCPC_REG_ALERT:
		payload[0] = pd.alert & 0xff;
		payload[1] = (pd.alert >> 8) & 0xff;
		return 2;
	case TCPC_REG_ALERT_MASK:
		payload[0] = pd.alert_mask & 0xff;
		payload[1] = (pd.alert_mask >> 8) & 0xff;
		return 2;
	case TCPC_REG_POWER_STATUS_MASK:
		payload[0] = pd.power_status_mask;
		return 1;

	// Control registers
	case TCPC_REG_TCPC_CTRL:
		payload[0] = pd.cc_tcpc_ctrl;
		return 1;
	case TCPC_REG_ROLE_CTRL:
		payload[0] = pd.cc_role_ctrl;
		return 1;
	case TCPC_REG_POWER_CTRL:
		payload[0] = pd.cc_power_ctrl;
		return 1;

	// Status registers
	case TCPC_REG_CC_STATUS:
		if (pd.internal_flags & TCPC_FLAG_LOOKING4CON) {
			payload[0] = TCPC_REG_CC_STATUS_LOOK4CONNECTION_MASK;
		} else {
			payload[0] = TCPC_REG_CC_STATUS_SET(
					(pd.internal_flags& TCPC_FLAG_CON_RESULT) ? 1 : 0,
					// If ((POWER_CONTROL.EnableVconn=1 and TCPC_CONTROL.PlugOrientation=1))
					// or ROLE_CONTROL.CC1=Ra or ROLE_CONTROL.CC1=open
					(TCPC_REG_POWER_CTRL_VCONN(pd.cc_power_ctrl) && TCPC_REG_TCPC_CTRL_POLARITY(pd.cc_tcpc_ctrl)) || (
						TCPC_REG_CC_STATUS_CC1(pd.cc_role_ctrl) == TYPEC_CC_RA ||
						TCPC_REG_CC_STATUS_CC1(pd.cc_role_ctrl) == TYPEC_CC_OPEN) ? 0 : pd.cc_status[0] & 0x3,
					// If ((POWER_CONTROL.EnableVconn=1 and TCPC_CONTROL.PlugOrientation=0))
					// or ROLE_CONTROL.CC2=Ra or ROLE_CONTROL.CC2=open
					(TCPC_REG_POWER_CTRL_VCONN(pd.cc_power_ctrl) && !TCPC_REG_TCPC_CTRL_POLARITY(pd.cc_tcpc_ctrl)) || (
						TCPC_REG_CC_STATUS_CC2(pd.cc_role_ctrl) == TYPEC_CC_RA ||
						TCPC_REG_CC_STATUS_CC2(pd.cc_role_ctrl) == TYPEC_CC_OPEN) ? 0 : pd.cc_status[1] & 0x3);
		}
		return 1;
	case TCPC_REG_POWER_STATUS:
		payload[0] = pd.power_status;
		return 1;


	case TCPC_REG_MSG_HDR_INFO:
		payload[0] = TCPC_REG_MSG_HDR_INFO_SET(pd.data_role, pd.power_role);
		return 1;
	case TCPC_REG_RX_DETECT:
		payload[0] = pd.reg_recv_detect;
		return 1;

	case TCPC_REG_RX_BYTE_CNT:
		payload[0] = 3 + 4 *
			PD_HEADER_CNT(pd.rx_head[pd.rx_buf_tail]);
		return 1;
	case TCPC_REG_RX_BUF_FRAME_TYPE:
		return 0;
	case TCPC_REG_RX_HDR:
		payload[0] = pd.rx_head[pd.rx_buf_tail] & 0xff;
		payload[1] = (pd.rx_head[pd.rx_buf_tail] >> 8) & 0xff;
		return 2;
	case TCPC_REG_RX_DATA:
		memcpy(payload, pd.rx_payload[pd.rx_buf_tail],
		       sizeof(pd.rx_payload[pd.rx_buf_tail]));
		return sizeof(pd.rx_payload[pd.rx_buf_tail]);

	case TCPC_REG_TX_HDR:
		payload[0] = pd.tx_head & 0xff;
		payload[1] = (pd.tx_head >> 8) & 0xff;
		return 2;
	case TCPC_REG_TX_DATA:
		memcpy(payload, pd.tx_payload,
		       sizeof(pd.tx_payload));
		return sizeof(pd.tx_payload);
	default:
		return 0;
	}
}

static inline void tcpc_detect_cc_status(uint64_t cur_timestamp)
{
	static uint8_t last_cc1, last_cc2;

	uint8_t cc1 = pd_cc_read_status(1, TCPC_REG_ROLE_CTRL_CC1(pd.cc_role_ctrl), TCPC_REG_ROLE_CTRL_RP(pd.cc_role_ctrl));
	uint8_t cc2 = pd_cc_read_status(2, TCPC_REG_ROLE_CTRL_CC2(pd.cc_role_ctrl), TCPC_REG_ROLE_CTRL_RP(pd.cc_role_ctrl));

	if (pd.internal_flags & TCPC_FLAG_DEBOUNCING) {
		if (last_cc1 != cc1 || last_cc2 != cc2) {
			// CC lines are still oscillating
			last_cc1 = cc1;
			last_cc2 = cc2;
			pd.cc_debouncing_timestamp = cur_timestamp;
		} else {
			// CC lines are not changing
			if (pd.internal_flags & TCPC_FLAG_LOOKING4CON) {
				pd.internal_flags &=~ TCPC_FLAG_DEBOUNCING;
				pd.cc_status[0] = last_cc1;
				pd.cc_status[1] = last_cc2;

				if ((pd.internal_flags & TCPC_FLAG_DRP_TOGGLE_AS_SNK) ||
							(TCPC_REG_CC_STATUS_CC2(pd.cc_role_ctrl)==TYPEC_CC_RD &&
							!(pd.internal_flags&TCPC_FLAG_DRP_TOGGLE_AS_SRC) &&
							!(pd.internal_flags&TCPC_FLAG_DRP_TOGGLE_AS_SNK))
						) {
					if (cc1 != TYPEC_CC_VOLT_OPEN
							|| cc2 != TYPEC_CC_VOLT_OPEN) {
						// Found potential connection as sink

						// CC.Looking4Connection=0, CC.ConnectResult=1
						pd.internal_flags &= ~(TCPC_FLAG_LOOKING4CON
								| TCPC_FLAG_DRP_TOGGLE_AS_SNK
								| TCPC_FLAG_CON_RESULT);
						pd.internal_flags |= TCPC_FLAG_CON_RESULT;
						alert(TCPC_REG_ALERT_CC_STATUS);
					}
				} else if ((pd.internal_flags & TCPC_FLAG_DRP_TOGGLE_AS_SRC) ||
							(TCPC_REG_CC_STATUS_CC2(pd.cc_role_ctrl)==TYPEC_CC_RP &&
							!(pd.internal_flags&TCPC_FLAG_DRP_TOGGLE_AS_SRC) &&
							!(pd.internal_flags&TCPC_FLAG_DRP_TOGGLE_AS_SNK))
						) {
					if (cc1 == TYPEC_CC_RD || cc2 == TYPEC_CC_RD
							|| (cc1 == TYPEC_CC_RA && cc2 == TYPEC_CC_RA)) {
						// Found potential connection as source

						// CC.Looking4Connection=0, CC.ConnectResult=0
						pd.internal_flags &= ~(TCPC_FLAG_LOOKING4CON
								| TCPC_FLAG_DRP_TOGGLE_AS_SRC
								| TCPC_FLAG_CON_RESULT);
						alert(TCPC_REG_ALERT_CC_STATUS);
					}
				} // There should not be other cases!
			} else {
				if (cur_timestamp > pd.cc_debouncing_timestamp + 10000) {
					pd.internal_flags &= ~ TCPC_FLAG_DEBOUNCING;

					// Now cc1 and cc2 contains the valid state
					if (pd.cc_status[0] != cc1 || pd.cc_status[1] != cc2) {
						// Found next state
						pd.cc_status[0] = last_cc1;
						pd.cc_status[1] = last_cc2;
						alert(TCPC_REG_ALERT_CC_STATUS);
					}
				}
			}
		}
	} else {
		if (pd.cc_status[0] != cc1 || pd.cc_status[1] != cc2) {
			// CC lines are different from previous valid state
			last_cc1 = cc1;
			last_cc2 = cc2;
			pd.internal_flags |= TCPC_FLAG_DEBOUNCING;
			pd.cc_debouncing_timestamp = cur_timestamp;
		} // if (pd.cc_status[0] != cc1 || pd.cc_status[1] != cc2) {
	}
}

// Run TCPC state machine once
void tcpc_run(void)
{
	uint64_t cur_timestamp = timestamp_get();

	if (pd.internal_flags & TCPC_FLAG_LOOKING4CON) {
		if (pd.internal_flags & TCPC_FLAG_DRP_TOGGLE_AS_SNK) {
			if (cur_timestamp > pd.drp_last_toggle_timestamp + TCPC_TIMER_TOGGLE_SNK) {
				// Timer expired, Switch to SRC (Rp)
				pd.internal_flags &=~ TCPC_FLAG_DRP_TOGGLE_AS_SNK;
				pd.internal_flags |= TCPC_FLAG_DRP_TOGGLE_AS_SRC;

				pd_cc_set(TCPC_REG_ROLE_CTRL_SET(0, TCPC_REG_ROLE_CTRL_RP(pd.cc_role_ctrl), TYPEC_CC_RP, TYPEC_CC_RP));
				pd.cc_status[0] = TYPEC_CC_VOLT_OPEN;
				pd.cc_status[1] = TYPEC_CC_VOLT_OPEN;
				pd.cc_last_sampled_timestamp = cur_timestamp;	// Postponed the next sampling of CC line, debouncing
			}
		} else if (pd.internal_flags & TCPC_FLAG_DRP_TOGGLE_AS_SRC) {
			if (cur_timestamp > pd.drp_last_toggle_timestamp + TCPC_TIMER_TOGGLE_SRC) {
				// Timer expired, Switch to SRC (Rp)
				pd.internal_flags &=~ TCPC_FLAG_DRP_TOGGLE_AS_SRC;
				pd.internal_flags |= TCPC_FLAG_DRP_TOGGLE_AS_SNK;

				pd_cc_set(TCPC_REG_ROLE_CTRL_SET(0, 0, TYPEC_CC_RD, TYPEC_CC_RD));
				pd.cc_status[0] = TYPEC_CC_VOLT_OPEN;
				pd.cc_status[1] = TYPEC_CC_VOLT_OPEN;
				pd.cc_last_sampled_timestamp = cur_timestamp;	// Postponed the next sampling of CC line, debouncing
			}
		}
	}


	// Check CC lines every 1mS
	if (cur_timestamp > pd.cc_last_sampled_timestamp + 1000)
	{
		pd.cc_last_sampled_timestamp = cur_timestamp;

		// Check CC lines
		tcpc_detect_cc_status(cur_timestamp);
	}

	if (TCPC_REG_RX_ENABLED(pd.reg_recv_detect))
		pd_rx_enable_monitoring();
}
