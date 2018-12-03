#ifndef __PD_DEF_H
#define __PD_DEF_H

/* build message header */
#define PD_HEADER(type, prole, drole, id, cnt, rev, ext) \
	((type) | ((rev) << 6) | \
	((drole) << 5) | ((prole) << 8) | \
	((id) << 9) | ((cnt) << 12) | ((ext) << 15))

/* Used for processing pd header */
#define PD_HEADER_EXT(header)  (((header) >> 15) & 1)
#define PD_HEADER_CNT(header)  (((header) >> 12) & 7)
#define PD_HEADER_TYPE(header) ((header) & 0xF)
#define PD_HEADER_ID(header)   (((header) >> 9) & 7)
#define PD_HEADER_REV(header)  (((header) >> 6) & 3)

/* Control Message type */
enum pd_ctrl_msg_type {
	/* 0 Reserved */
	PD_CTRL_GOOD_CRC = 1,
	PD_CTRL_GOTO_MIN = 2,
	PD_CTRL_ACCEPT = 3,
	PD_CTRL_REJECT = 4,
	PD_CTRL_PING = 5,
	PD_CTRL_PS_RDY = 6,
	PD_CTRL_GET_SOURCE_CAP = 7,
	PD_CTRL_GET_SINK_CAP = 8,
	PD_CTRL_DR_SWAP = 9,
	PD_CTRL_PR_SWAP = 10,
	PD_CTRL_VCONN_SWAP = 11,
	PD_CTRL_WAIT = 12,
	PD_CTRL_SOFT_RESET = 13,
	/* 14-15 Reserved */

	/* Used for REV 3.0 */
	PD_CTRL_NOT_SUPPORTED = 16,
	PD_CTRL_GET_SOURCE_CAP_EXT = 17,
	PD_CTRL_GET_STATUS = 18,
	PD_CTRL_FR_SWAP = 19,
	PD_CTRL_GET_PPS_STATUS = 20,
	PD_CTRL_GET_COUNTRY_CODES = 21,
	/* 22-31 Reserved */
};

/* Extended message type for REV 3.0 */
enum pd_ext_msg_type {
	/* 0 Reserved */
	PD_EXT_SOURCE_CAP = 1,
	PD_EXT_STATUS = 2,
	PD_EXT_GET_BATTERY_CAP = 3,
	PD_EXT_GET_BATTERY_STATUS = 4,
	PD_EXT_BATTERY_CAP = 5,
	PD_EXT_GET_MANUFACTURER_INFO = 6,
	PD_EXT_MANUFACTURER_INFO = 7,
	PD_EXT_SECURITY_REQUEST = 8,
	PD_EXT_SECURITY_RESPONSE = 9,
	PD_EXT_FIRMWARE_UPDATE_REQUEST = 10,
	PD_EXT_FIRMWARE_UPDATE_RESPONSE = 11,
	PD_EXT_PPS_STATUS = 12,
	PD_EXT_COUNTRY_INFO = 13,
	PD_EXT_COUNTRY_CODES = 14,
	/* 15-31 Reserved */
};

/* Data message type */
enum pd_data_msg_type {
	/* 0 Reserved */
	PD_DATA_SOURCE_CAP = 1,
	PD_DATA_REQUEST = 2,
	PD_DATA_BIST = 3,
	PD_DATA_SINK_CAP = 4,
	/* 5-14 Reserved for REV 2.0 */
	PD_DATA_BATTERY_STATUS = 5,
	PD_DATA_ALERT = 6,
	PD_DATA_GET_COUNTRY_INFO = 7,
	/* 8-14 Reserved for REV 3.0 */
	PD_DATA_VENDOR_DEF = 15,
};

/* Protocol revision */
#define PD_REV10 0
#define PD_REV20 1
#define PD_REV30 2

/* Power role */
#define PD_ROLE_SINK   0
#define PD_ROLE_SOURCE 1
/* Data role */
#define PD_ROLE_UFP    0
#define PD_ROLE_DFP    1
/* Vconn role */
#define PD_ROLE_VCONN_OFF 0
#define PD_ROLE_VCONN_ON  1

/* chunk is a request or response in REV 3.0 */
#define CHUNK_RESPONSE 0
#define CHUNK_REQUEST  1

/* collision avoidance Rp values in REV 3.0 */
#define SINK_TX_OK TYPEC_RP_3A0
#define SINK_TX_NG TYPEC_RP_1A5

/* Minimum PD supply current  (mA) */
#define PD_MIN_MA	500

/* Minimum PD voltage (mV) */
#define PD_MIN_MV	5000



/* Voltage threshold to detect connection when presenting Rd */
#define PD_SNK_VA_MV             250

/* --- Protocol layer functions --- */

enum pd_states {
	PD_STATE_DISABLED,
	PD_STATE_SUSPENDED,

	PD_STATE_SNK_DISCONNECTED,
	PD_STATE_SNK_DISCONNECTED_DEBOUNCE,
	PD_STATE_SNK_HARD_RESET_RECOVER,
	PD_STATE_SNK_DISCOVERY,
	PD_STATE_SNK_REQUESTED,
	PD_STATE_SNK_TRANSITION,
	PD_STATE_SNK_READY,

	PD_STATE_SNK_SWAP_INIT,
	PD_STATE_SNK_SWAP_SNK_DISABLE,
	PD_STATE_SNK_SWAP_SRC_DISABLE,
	PD_STATE_SNK_SWAP_STANDBY,
	PD_STATE_SNK_SWAP_COMPLETE,


	PD_STATE_SRC_DISCONNECTED,
	PD_STATE_SRC_DISCONNECTED_DEBOUNCE,
	PD_STATE_SRC_HARD_RESET_RECOVER,
	PD_STATE_SRC_STARTUP,
	PD_STATE_SRC_DISCOVERY,
	PD_STATE_SRC_NEGOCIATE,
	PD_STATE_SRC_ACCEPTED,
	PD_STATE_SRC_POWERED,
	PD_STATE_SRC_TRANSITION,
	PD_STATE_SRC_READY,
	PD_STATE_SRC_GET_SINK_CAP,
	PD_STATE_DR_SWAP,


	PD_STATE_SRC_SWAP_INIT,
	PD_STATE_SRC_SWAP_SNK_DISABLE,
	PD_STATE_SRC_SWAP_SRC_DISABLE,
	PD_STATE_SRC_SWAP_STANDBY,


	PD_STATE_VCONN_SWAP_SEND,
	PD_STATE_VCONN_SWAP_INIT,
	PD_STATE_VCONN_SWAP_READY,

	PD_STATE_SOFT_RESET,
	PD_STATE_HARD_RESET_SEND,
	PD_STATE_HARD_RESET_EXECUTE,

	PD_STATE_BIST_RX,
	PD_STATE_BIST_TX,


	PD_STATE_DRP_AUTO_TOGGLE,

	/* Number of states. Not an actual state. */
	PD_STATE_COUNT,
};

// Protocol layer internal flags
#define PD_FLAGS_PING_ENABLED      (1 << 0) /* SRC_READY pings enabled */
#define PD_FLAGS_PARTNER_DR_POWER  (1 << 1) /* port partner is dualrole power */
#define PD_FLAGS_PARTNER_DR_DATA   (1 << 2) /* port partner is dualrole data */
#define PD_FLAGS_CHECK_IDENTITY    (1 << 3) /* discover identity in READY */
#define PD_FLAGS_SNK_CAP_RECVD     (1 << 4) /* sink capabilities received */
#define PD_FLAGS_TCPC_DRP_TOGGLE   (1 << 5) /* TCPC-controlled DRP toggling */
#define PD_FLAGS_EXPLICIT_CONTRACT (1 << 6) /* explicit pwr contract in place */
#define PD_FLAGS_VBUS_NEVER_LOW    (1 << 7) /* VBUS input has never been low */
#define PD_FLAGS_PREVIOUS_PD_CONN  (1 << 8) /* previously PD connected */
#define PD_FLAGS_CHECK_PR_ROLE     (1 << 9) /* check power role in READY */
#define PD_FLAGS_CHECK_DR_ROLE     (1 << 10)/* check data role in READY */
#define PD_FLAGS_PARTNER_EXTPOWER  (1 << 11)/* port partner has external pwr */
#define PD_FLAGS_VCONN_ON          (1 << 12)/* vconn is being sourced */
#define PD_FLAGS_TRY_SRC           (1 << 13)/* Try.SRC states are active */
#define PD_FLAGS_PARTNER_USB_COMM  (1 << 14)/* port partner is USB comms */
#define PD_FLAGS_UPDATE_SRC_CAPS   (1 << 15)/* send new source capabilities */
#define PD_FLAGS_TS_DTS_PARTNER    (1 << 16)/* partner has rp/rp or rd/rd */
/* Flags to clear on a disconnect */
#define PD_FLAGS_RESET_ON_DISCONNECT_MASK (PD_FLAGS_PARTNER_DR_POWER | \
					   PD_FLAGS_PARTNER_DR_DATA | \
					   PD_FLAGS_CHECK_IDENTITY | \
					   PD_FLAGS_SNK_CAP_RECVD | \
					   PD_FLAGS_TCPC_DRP_TOGGLE | \
					   PD_FLAGS_EXPLICIT_CONTRACT | \
					   PD_FLAGS_PREVIOUS_PD_CONN | \
					   PD_FLAGS_CHECK_PR_ROLE | \
					   PD_FLAGS_CHECK_DR_ROLE | \
					   PD_FLAGS_PARTNER_EXTPOWER | \
					   PD_FLAGS_VCONN_ON | \
					   PD_FLAGS_TRY_SRC | \
					   PD_FLAGS_PARTNER_USB_COMM | \
					   PD_FLAGS_UPDATE_SRC_CAPS | \
					   PD_FLAGS_TS_DTS_PARTNER)

enum pd_cc_states {
	PD_CC_NONE,

	/* From DFP perspective */
	PD_CC_NO_UFP,
	PD_CC_AUDIO_ACC,
	PD_CC_DEBUG_ACC,
	PD_CC_UFP_ATTACHED,

	/* From UFP perspective */
	PD_CC_DFP_ATTACHED
};

/* Timers */
#define MSEC 1000	// 1ms = 1000uS
#define PD_T_SINK_TX          (18*MSEC) /* between 16ms and 20 */
#define PD_T_CHUNK_SENDER_RSP (24*MSEC) /* between 24ms and 30ms */
#define PD_T_CHUNK_SENDER_REQ (24*MSEC) /* between 24ms and 30ms */
#define PD_T_SEND_SOURCE_CAP  (100*MSEC) /* between 100ms and 200ms */
#define PD_T_SINK_WAIT_CAP    (600*MSEC) /* between 310ms and 620ms */
#define PD_T_SINK_TRANSITION   (35*MSEC) /* between 20ms and 35ms */
#define PD_T_SOURCE_ACTIVITY   (45*MSEC) /* between 40ms and 50ms */
#define PD_T_SENDER_RESPONSE   (30*MSEC) /* between 24ms and 30ms */
#define PD_T_PS_TRANSITION    (500*MSEC) /* between 450ms and 550ms */
#define PD_T_PS_SOURCE_ON     (480*MSEC) /* between 390ms and 480ms */
#define PD_T_PS_SOURCE_OFF    (920*MSEC) /* between 750ms and 920ms */
#define PD_T_PS_HARD_RESET     (25*MSEC) /* between 25ms and 35ms */
#define PD_T_ERROR_RECOVERY    (25*MSEC) /* 25ms */
#define PD_T_CC_DEBOUNCE       (100*MSEC) /* between 100ms and 200ms */
/* DRP_SNK + DRP_SRC must be between 50ms and 100ms with 30%-70% duty cycle */
#define PD_T_DRP_SNK           (40*MSEC) /* toggle time for sink DRP */
#define PD_T_DRP_SRC           (30*MSEC) /* toggle time for source DRP */
#define PD_T_DEBOUNCE          (15*MSEC) /* between 10ms and 20ms */
#define PD_T_SINK_ADJ          (55*MSEC) /* between PD_T_DEBOUNCE and 60ms */
#define PD_T_SRC_RECOVER      (760*MSEC) /* between 660ms and 1000ms */
#define PD_T_SRC_RECOVER_MAX (1000*MSEC) /* 1000ms */
#define PD_T_SRC_TURN_ON      (275*MSEC) /* 275ms */
#define PD_T_SAFE_0V          (650*MSEC) /* 650ms */
#define PD_T_NO_RESPONSE     (5500*MSEC) /* between 4.5s and 5.5s */
#define PD_T_BIST_TRANSMIT     (50*MSEC) /* 50ms (used for task_wait arg) */
#define PD_T_BIST_RECEIVE      (60*MSEC) /* 60ms (max time to process bist) */
#define PD_T_VCONN_SOURCE_ON  (100*MSEC) /* 100ms */
#define PD_T_TRY_SRC          (125*MSEC) /* Max time for Try.SRC state */
#define PD_T_TRY_WAIT         (600*MSEC) /* Max time for TryWait.SNK state */
#define PD_T_SINK_REQUEST     (100*MSEC) /* Wait 100ms before next request */

#endif
