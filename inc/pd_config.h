/* USB Type-C Port PHY Configurations */
/*
 *	If defined, the PHY will detect all SOPs defined in PD3.0.
 *	Otherwise, it only detects SOP and Hard Reset.
 */
#define CONFIG_PD_DETECT_ALL_SOP

/* USB Power delivery module*/
//#define CONFIG_USB_PD_HANDLE_TCPC_RESET	// Apply to TCPCi only
//#define CONFIG_USBC_VCONN					// Capable of sourcing VConn, source-capable device only
//#define CONFIG_USB_PD_USE_VDM				// Enable VDM reception and transmission
//#define CONFIG_USB_PD_ALT_MODE			// Support alternate mode, need CONFIG_USB_PD_USE_VDM
#define CONFIG_USB_PD_FUNC_SNK			// Can act as a sink
// CONFIG_USB_PD_STD_SNK_PWR_MGR			// Use Google's power profile selection method on sink
	// Need to define PD_MAX_POWER_MW, PD_MAX_CURRENT_MA, PD_MAX_VOLTAGE_MV
//#define CONFIG_USD_PD_FUNC_SRC			// Can act as a source
//#define CONFIG_USB_PD_DR_SWAP				// Support DR_SWAP


#if defined(CONFIG_USB_PD_ALT_MODE) && !defined(CONFIG_USB_PD_USE_VDM)
#error "CONFIG_USB_PD_USE_VDM required CONFIG_USB_PD_ALT_MODE!"
#endif

#if defined(CONFIG_USB_PD_FUNC_SNK) && defined(CONFIG_USD_PD_FUNC_SRC)
#define CONFIG_USB_PD_DUAL_ROLE 1
#endif
