#ifndef __PD_DEF_H
#define __PD_DEF_H

#define PD_REV 1	// PD2.0 spec.

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

/* Default retry count for transmitting */
#define PD_RETRY_COUNT 3

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

/* Minimum PD supply current  (mA) */
#define PD_MIN_MA	500

/* Minimum PD voltage (mV) */
#define PD_MIN_MV	5000

/* No connect voltage threshold for sources based on Rp */
#define PD_SRC_DEF_VNC_MV        1600
#define PD_SRC_1_5_VNC_MV        1600
#define PD_SRC_3_0_VNC_MV        2600

/* Rd voltage threshold for sources based on Rp */
#define PD_SRC_DEF_RD_THRESH_MV  200
#define PD_SRC_1_5_RD_THRESH_MV  400
#define PD_SRC_3_0_RD_THRESH_MV  800

/* Voltage threshold to detect connection when presenting Rd */
#define PD_SNK_VA_MV             250

#endif
