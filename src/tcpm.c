#include "tcpm.h"
#include "tcpci.h"
#include "pd.h"

static uint8_t selected_rp;

static inline void tcpc_write(uint8_t reg, uint8_t val)
{
	tcpc_i2c_write(reg, 1, &val);
}

static inline void tcpc_write16(uint8_t reg, uint16_t val)
{
	tcpc_i2c_write(reg, 2, (uint8_t*)&val);
}

#define tcpc_read(reg, val)  tcpc_i2c_read(reg, val)

void tcpm_get_cc(int port, int* cc1, int* cc2)
{
	uint8_t status;
	tcpc_read(TCPC_REG_CC_STATUS, &status);

	*cc1 = TCPC_REG_CC_STATUS_CC1(status);
	*cc2 = TCPC_REG_CC_STATUS_CC2(status);

	/*
	 * If status is not open, then OR in termination to convert to
	 * enum tcpc_cc_voltage_status.
	 */
	if (*cc1 != TYPEC_CC_VOLT_OPEN)
		*cc1 |= TCPC_REG_CC_STATUS_TERM(status) << 2;
	if (*cc2 != TYPEC_CC_VOLT_OPEN)
		*cc2 |= TCPC_REG_CC_STATUS_TERM(status) << 2;
}

#ifdef CONFIG_USB_PD_VBUS_DETECT_TCPC
int tcpm_get_vbus_level(int port)
{
	uint8_t reg;

	/* Read Power Status register */
	tcpc_read(TCPC_REG_POWER_STATUS, &reg);
	/* Update VBUS status */
	return reg & TCPC_REG_POWER_STATUS_VBUS_PRES ? 1 : 0;
}
#endif

void tcpm_select_rp_value(int port, int rp)
{
	selected_rp = rp;
}

void tcpm_set_cc(int port, int pull)
{
	/* Set manual control, and set both CC lines to the same pull */
	return tcpc_write(TCPC_REG_ROLE_CTRL, TCPC_REG_ROLE_CTRL_SET(0, selected_rp, pull, pull));
}

void tcpm_set_polarity(int port, int polarity)
{
	tcpc_write(TCPC_REG_TCPC_CTRL, TCPC_REG_TCPC_CTRL_SET(polarity));
}

void tcpm_set_vconn(int port, int enable)
{
	uint8_t reg;

	tcpc_read(TCPC_REG_POWER_CTRL, &reg);

	reg &= ~TCPC_REG_POWER_CTRL_VCONN(1);
	reg |= TCPC_REG_POWER_CTRL_VCONN(enable);
	tcpc_write(TCPC_REG_POWER_CTRL, reg);
}

void tcpm_set_msg_header(int port, int power_role, int data_role)
{
	tcpc_write(TCPC_REG_MSG_HDR_INFO, TCPC_REG_MSG_HDR_INFO_SET(data_role, power_role, PD_REV20));
}

void tcpm_set_rx_enable(int port, int enable)
{
	/* If enable, then set RX detect for SOP and HRST */
	tcpc_write(TCPC_REG_RX_DETECT, enable ? TCPC_REG_RX_DETECT_SOP_HRST_MASK : 0);
}

int tcpm_get_message(int port, uint32_t *payload, int *head)
{
	uint8_t cnt;
	tcpc_read(TCPC_REG_RX_BYTE_CNT, &cnt);
	uint16_t header;
	tcpc_read(TCPC_REG_RX_HDR, (uint8_t*)&header);
	*head = header;
	if (cnt>3)
		tcpc_read(TCPC_REG_RX_DATA, (uint8_t*)payload);

	/* Read complete, clear RX status alert bit */
	tcpc_write16(TCPC_REG_ALERT, TCPC_REG_ALERT_RX_STATUS);

	return 0;
}

void tcpm_transmit(int port, enum tcpm_transmit_type type, uint16_t header, const uint32_t *data)
{
	uint8_t cnt = PD_HEADER_CNT(header) << 2;

	tcpc_write16(TCPC_REG_TX_HDR, header);

	if (cnt > 0)
		tcpc_i2c_write(TCPC_REG_TX_DATA, cnt, (uint8_t*)data);

	tcpc_write(TCPC_REG_TRANSMIT, TCPC_REG_TRANSMIT_SET(type));
}

#ifdef CONFIG_USB_PD_DISCHARGE_TCPC
void tcpc_discharge_vbus(int port, int enable)
{
	uint8_t reg;

	if (tcpc_read(TCPC_REG_POWER_CTRL, &reg) <= 0)
		return;

	if (enable)
		reg |= TCPC_REG_POWER_CTRL_FORCE_DISCHARGE;
	else
		reg &= ~TCPC_REG_POWER_CTRL_FORCE_DISCHARGE;

	tcpc_write(TCPC_REG_POWER_CTRL, reg);
}
#endif

#ifdef CONFIG_USB_PD_DUAL_ROLE_AUTO_TOGGLE
void tcpm_set_drp_toggle(int port, int enable)
{
	if (!enable)
		return;

	/* Set auto drp toggle */
	tcpc_write(TCPC_REG_ROLE_CTRL, TCPC_REG_ROLE_CTRL_SET(1, TYPEC_RP_USB, TYPEC_CC_OPEN, TYPEC_CC_OPEN));

	/* Set Look4Connection command */
	tcpc_write(TCPC_REG_COMMAND, TCPC_REG_COMMAND_LOOK4CONNECTION);

	return;
}
#endif

void tcpc_alert(int port)
{
	uint16_t status;

	/* Read the Alert register from the TCPC */
	tcpc_i2c_read(TCPC_REG_ALERT, (uint8_t*)&status);

	/*
	 * Clear alert status for everything except RX_STATUS, which shouldn't
	 * be cleared until we have successfully retrieved message.
	 */
	if (status & ~TCPC_REG_ALERT_RX_STATUS) {
		uint16_t new_val = status & ~TCPC_REG_ALERT_RX_STATUS;
		tcpc_i2c_write(TCPC_REG_ALERT, 2, (uint8_t*) &new_val);
	}

	if (status & TCPC_REG_ALERT_CC_STATUS) {
		/* CC status changed, wake task */
		task_set_event(PD_PORT_TO_TASK_ID(port), PD_EVENT_CC, 0);
	}

	if (status & TCPC_REG_ALERT_POWER_STATUS) {
		uint8_t reg = 0;

		tcpc_read(TCPC_REG_POWER_STATUS_MASK, &reg);

		if (reg == TCPC_REG_POWER_STATUS_MASK_ALL) {
			/*
			 * If power status mask has been reset, then the TCPC
			 * has reset.
			 */
			task_set_event(PD_PORT_TO_TASK_ID(port), PD_EVENT_TCPC_RESET, 0);
#if defined(CONFIG_USB_PD_VBUS_DETECT_TCPC) && defined(CONFIG_USB_CHARGER)
		} else {
			/* Read Power Status register */
			tcpm_get_power_status(port, &reg);
			/* Update charge manager with new VBUS state */
			usb_charger_vbus_change(port,
				reg & TCPC_REG_POWER_STATUS_VBUS_PRES);
			task_set_event(PD_PORT_TO_TASK_ID(port), TASK_EVENT_WAKE, 0);
#endif /* CONFIG_USB_PD_VBUS_DETECT_TCPC && CONFIG_USB_CHARGER */
		}
	}
	if (status & TCPC_REG_ALERT_RX_STATUS) {
		/* message received */
		task_set_event(PD_PORT_TO_TASK_ID(port), PD_EVENT_RX, 0);
	}
	if (status & TCPC_REG_ALERT_RX_HARD_RST) {
		/* hard reset received */
		pd_execute_hard_reset(port);
		task_set_event(PD_PORT_TO_TASK_ID(port), TASK_EVENT_WAKE, 0);
	}
	if (status & TCPC_REG_ALERT_TX_COMPLETE) {
		/* transmit complete */
		pd_transmit_complete(port, status & TCPC_REG_ALERT_TX_SUCCESS ? TCPC_TX_COMPLETE_SUCCESS : TCPC_TX_COMPLETE_FAILED);
	}
}

/*
 * On TCPC i2c failure, make 30 tries (at least 300ms) before giving up
 * in order to allow the TCPC time to boot / reset.
 */

int tcpm_init(int port)
{
//	uint8_t power_status;

// 	TCPC_REG_POWER_STATUS_UNINIT should not be observed since our tcpc is always initialized first
//	tcpc_read(TCPC_REG_POWER_STATUS, &power_status);
//		/*
//		 * If read succeeds and the uninitialized bit is clear, then
//		 * initalization is complete, clear all alert bits and write
//		 * the initial alert mask.
//		 */
//	if (!(power_status & TCPC_REG_POWER_STATUS_UNINIT))
//		return -1;


	/* Set Power Status mask */
#ifdef CONFIG_USB_PD_VBUS_DETECT_TCPC
	tcpc_write(TCPC_REG_POWER_STATUS_MASK , TCPC_REG_POWER_STATUS_VBUS_PRES);
#else
	tcpc_write(TCPC_REG_POWER_STATUS_MASK , 0);
#endif

	/* Clear alert request and set alert mask */
	tcpc_write16(TCPC_REG_ALERT, 0xffff);

	/* Initialize power_status_mask */

	/* Set the alert mask in TCPC */
	tcpc_write16(TCPC_REG_ALERT_MASK,
			/*
			 * Create mask of alert events that will cause the TCPC to
			 * signal the TCPM via the Alert# gpio line.
			 */
					TCPC_REG_ALERT_TX_SUCCESS | TCPC_REG_ALERT_TX_FAILED |
					TCPC_REG_ALERT_TX_DISCARDED | TCPC_REG_ALERT_RX_STATUS |
					TCPC_REG_ALERT_RX_HARD_RST | TCPC_REG_ALERT_CC_STATUS
#ifdef CONFIG_USB_PD_VBUS_DETECT_TCPC
					| TCPC_REG_ALERT_POWER_STATUS
#endif
			);

	/* Read chip info here when we know the chip is awake. */
	tcpm_get_chip_info(port, 1, NULL);

	return EC_SUCCESS;
}
