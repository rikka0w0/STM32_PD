#include "platform.h"
#include "pd_phy.h"
#include "tcpci.h"

/*
 * This file handles all CC pin functions except BMC communication:
 * 1. CC Rp & Rd advertising
 * 2. CC status detection
 * 3. Vconn control
 */

/*
 * Configurations
 * 1. PD_CFG_VCONN_CTRL can be disabled when there's no need to supply Vconn. e.g. sink only applications,
 *        Vconn is required for DRPs and Sources(except power adapter with a male type-c connector and a non-removable cable)
 * 2. PD_CFG_RP_3A0 can be disabled in source only applications, and then connect two 4.7k resistor between CCx and 3.3V
 * 3. PD_CFG_RP_1A5 and PD_CFG_RP_DEF can be disabled if no longer needed
 * 4. PD_CFG_RD_CTRL can be disabled in sink only applications, and then connect two 5.1k resistor between CCx and ground
 *
 * Spare GPIOs can be used for other purpose when it is not controlled by this file
 * Comment the following macros to disable corresponding functions
 */
#define PD_CFG_VCONN_CTRL	// Enable Vconn control			PF0, PF1
#define PD_CFG_RP_3A0		// Enable 5V3A Rp (4.7k)		PB5, PA3
//#define PD_CFG_RP_1A5		// Enable 5V1.5A Rp (12k)		PA11, PA12
//#define PD_CFG_RP_DEF		// Enable 5V0.5A Rp (36k)		PA9, PA10
#define PD_CFG_RD_CTRL		// Enable Rd (5.1k) control		PB3, PA15
#define PD_CFG_VBUS_DECT	// Detect VBus

// UFP CC line threshold value (Across Rd)
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


// DFP CC line threshold value (Rp to ground)
//	Rp Configuration	Ra->Rd	Rd->NC
//	Default				0.2V	1.6V
//	1.5A				0.4V	1.6V
//	3.0A				0.8V	2.6V
#define  PD_SRC_DEF_RARD	248
#define	 PD_SRC_DEF_RDNC	1986
#define  PD_SRC_1A5_RARD	496
#define	 PD_SRC_1A5_RDNC	1986
#define  PD_SRC_3A0_RARD	993
#define	 PD_SRC_3A0_RDNC	3227

/*
 * Read the status of a given cc wire (1 or 2), if resistor is TYPEC_CC_RP then rp must be specified
 */
uint8_t pd_cc_read_status(uint8_t cc, uint8_t resistor, uint8_t rp)
{
	uint16_t adc_val;
	if (cc == PD_CC_1) {
		adc_val = adc_read(PD_CC1_PIN);
	} else if (cc == PD_CC_2) {
		adc_val = adc_read(PD_CC2_PIN);
	} else {
		return TYPEC_CC_VOLT_OPEN;
	}

	if (resistor == TYPEC_CC_RD) {	// We are sink
		if((adc_val >= PD_RDV_3A0_MIN) && (adc_val <= PD_RDV_3A0_MAX))
			return TYPEC_CC_VOLT_SNK_3_0;
		else if((adc_val >= PD_RDV_1A5_MIN) && (adc_val <= PD_RDV_1A5_MAX))
			return TYPEC_CC_VOLT_SNK_1_5;
		else if((adc_val >= PD_RDV_DEFAULT_MIN) && (adc_val <= PD_RDV_DEFAULT_MAX))
			return TYPEC_CC_VOLT_SNK_DEF;
		else
			return TYPEC_CC_VOLT_OPEN;
	} else if (resistor == TYPEC_CC_RP) {	// We are source
		if (rp == TYPEC_RP_USB)	{
			if(adc_val >= PD_SRC_DEF_RDNC)
				return TYPEC_CC_VOLT_OPEN;
			else if(adc_val >= PD_SRC_DEF_RARD)
				return TYPEC_CC_VOLT_RD;
			else
				return TYPEC_CC_VOLT_RA;
		} else if (rp == TYPEC_RP_1A5) {
			if(adc_val >= PD_SRC_1A5_RDNC)
				return TYPEC_CC_VOLT_OPEN;
			else if(adc_val >= PD_SRC_1A5_RARD)
				return TYPEC_CC_VOLT_RD;
			else
				return TYPEC_CC_VOLT_RA;
		} else if (rp == TYPEC_RP_3A0) {
			if(adc_val >= PD_SRC_3A0_RDNC)
				return TYPEC_CC_VOLT_OPEN;
			else if(adc_val >= PD_SRC_3A0_RARD)
				return TYPEC_CC_VOLT_RD;
			else
				return TYPEC_CC_VOLT_RA;
		}
	}

	return TYPEC_CC_VOLT_OPEN;
}

// Vconn
#define PD_CC1_VCONN_DIS() (GPIOF->ODR |= (1<<(1*0)))
#define PD_CC1_VCONN_EN() (GPIOF->ODR &=~ (1<<(1*0)))
#define PD_CC2_VCONN_DIS() (GPIOF->ODR |= (1<<(1*1)))
#define PD_CC2_VCONN_EN() (GPIOF->ODR &=~ (1<<(1*1)))

// Rp
#define PD_CC1_RP_3A0_DIS() (GPIOB->MODER &=~ (3<<(2*5)))	// Set to input
#define PD_CC1_RP_3A0_EN() (GPIOB->MODER |= (1<<(2*5)))		// Set to output, strong pull-up
#define PD_CC2_RP_3A0_DIS() (GPIOA->MODER &=~ (3<<(2*3)))	// Set to input
#define PD_CC2_RP_3A0_EN() (GPIOA->MODER |= (1<<(2*3)))		// Set to output, strong pull-up

#define PD_CC1_RP_1A5_DIS() (GPIOA->MODER &=~ (3<<(2*11)))	// Set to input
#define PD_CC1_RP_1A5_EN() (GPIOA->MODER |= (1<<(2*11)))	// Set to output, strong pull-up
#define PD_CC2_RP_1A5_DIS() (GPIOA->MODER &=~ (3<<(2*12)))	// Set to input
#define PD_CC2_RP_1A5_EN() (GPIOA->MODER |= (1<<(2*12)))	// Set to output, strong pull-up

#define PD_CC1_RP_DEF_DIS() (GPIOA->MODER &=~ (3<<(2*9)))	// Set to input
#define PD_CC1_RP_DEF_EN() (GPIOA->MODER |= (1<<(2*9)))		// Set to output, strong pull-up
#define PD_CC2_RP_DEF_DIS() (GPIOA->MODER &=~ (3<<(2*10)))	// Set to input
#define PD_CC2_RP_DEF_EN() (GPIOA->MODER |= (1<<(2*10)))	// Set to output, strong pull-up

// Rd
#define PD_CC1_RD_DIS() (GPIOB->ODR |= (1<<(1*3)))
#define PD_CC1_RD_EN() (GPIOB->ODR &=~ (1<<(1*3)))
#define PD_CC2_RD_DIS() (GPIOA->ODR |= (1<<(1*15)))
#define PD_CC2_RD_EN() (GPIOA->ODR &=~ (1<<(1*15)))

void pd_cc_rprp_init(void)
{
#ifdef PD_CFG_VBUS_DECT
	GPIOA->MODER |= (3<<(2*0));		// Set to analog mode
	GPIOA->PUPDR &=~ (3<<(2*0));	// No pull-up/pull-down
#endif

#ifdef PD_CFG_VCONN_CTRL	// PF0, PF1
	GPIOF->MODER &=~ (3<<(2*0));	// Set to input
	GPIOF->OTYPER |= (1<<(1*0));	// Open drain
	GPIOF->OSPEEDR |= (3<<(2*0));	// High speed
	GPIOF->PUPDR &=~ (3<<(2*0));	// No pull-up/pull-down
	PD_CC1_VCONN_DIS();				// Set to Hiz, disable Vconn at first
	GPIOF->MODER |= (1<<(2*0));		// Set to output

	GPIOF->MODER &=~ (3<<(2*1));	// Set to input
	GPIOF->OTYPER |= (1<<(1*1));	// Open drain
	GPIOF->OSPEEDR |= (3<<(2*1));	// High speed
	GPIOF->PUPDR &=~ (3<<(2*1));	// No pull-up/pull-down
	PD_CC2_VCONN_DIS();				// Set to Hiz, disable Vconn at first
	GPIOF->MODER |= (1<<(2*1));		// Set to output
#endif // PD_CFG_VCONN_CTRL

#ifdef PD_CFG_RP_3A0		// PB5, PA3
	PD_CC1_RP_3A0_DIS();			// Set to input
	GPIOB->OTYPER &=~ (1<<(1*5));	// Push-pull
	GPIOB->OSPEEDR |= (3<<(2*5));	// High speed
	GPIOB->PUPDR &=~ (3<<(2*5));	// No pull-up/pull-down
	GPIOB->ODR |= (1<<(1*5));		// Set to 1

	PD_CC2_RP_3A0_DIS();			// Set to input
	GPIOA->OTYPER &=~ (1<<(1*3));	// Push-pull
	GPIOA->OSPEEDR |= (3<<(2*3));	// High speed
	GPIOA->PUPDR &=~ (3<<(2*3));	// No pull-up/pull-down
	GPIOA->ODR |= (1<<(1*3));		// Set to 1
#endif // PD_CFG_RP_3A0

#ifdef PD_CFG_RP_1A5		// PA11, PA12
	PD_CC1_RP_1A5_DIS();	// Set to input
	GPIOA->OTYPER &=~ (1<<(1*11));	// Push-pull
	GPIOA->OSPEEDR |= (3<<(2*11));	// High speed
	GPIOA->PUPDR &=~ (3<<(2*11));	// No pull-up/pull-down
	GPIOA->ODR |= (1<<(1*11));		// Set to 1

	PD_CC2_RP_1A5_DIS();	// Set to input
	GPIOA->OTYPER &=~ (1<<(1*12));	// Push-pull
	GPIOA->OSPEEDR |= (3<<(2*12));	// High speed
	GPIOA->PUPDR &=~ (3<<(2*12));	// No pull-up/pull-down
	GPIOA->ODR |= (1<<(1*12));		// Set to 1
#endif // PD_CFG_RP_1A5

#ifdef PD_CFG_RP_DEF		// PA9, PA10
	PD_CC1_RP_DEF_DIS();	// Set to input
	GPIOA->OTYPER &=~ (1<<(1*9));	// Push-pull
	GPIOA->OSPEEDR |= (3<<(2*9));	// High speed
	GPIOA->PUPDR &=~ (3<<(2*9));	// No pull-up/pull-down
	GPIOA->ODR |= (1<<(1*9));		// Set to 1

	PD_CC2_RP_DEF_DIS();	// Set to input
	GPIOA->OTYPER &=~ (1<<(1*10));	// Push-pull
	GPIOA->OSPEEDR |= (3<<(2*10));	// High speed
	GPIOA->PUPDR &=~ (3<<(2*10));	// No pull-up/pull-down
	GPIOA->ODR |= (1<<(1*10));		// Set to 1
#endif // PD_CFG_RP_DEF

#ifdef PD_CFG_RD_CTRL		// PB3, PA15
	GPIOB->MODER &=~ (3<<(2*3));	// Set to input
	GPIOB->OTYPER &=~ (1<<(1*3));	// Push-pull
	GPIOB->OSPEEDR |= (3<<(2*3));	// High speed
	GPIOB->PUPDR &=~ (3<<(2*3));	// No pull-up/pull-down
	PD_CC1_RD_EN();					// Set to Hiz, enable Rd at first (dead battery support)
	GPIOB->MODER |= (1<<(2*3));		// Set to output

	GPIOA->MODER &=~ (3<<(2*15));	// Set to input
	GPIOA->OTYPER &=~ (1<<(1*15));	// Push-pull
	GPIOA->OSPEEDR |= (3<<(2*15));	// High speed
	GPIOA->PUPDR &=~ (3<<(2*15));	// No pull-up/pull-down
	PD_CC2_RD_EN();					// Set to Hiz, enable Rd at first (dead battery support)
	GPIOA->MODER |= (1<<(2*15));	// Set to output
#endif // PD_CFG_RD_CTRL
}

void pd_cc_set(uint8_t role_ctrl_regval)
{
	// Disable Rp and Rd
#ifdef PD_CFG_RP_3A0
	PD_CC1_RP_3A0_DIS();
	PD_CC2_RP_3A0_DIS();
#endif
#ifdef PD_CFG_RP_1A5
	PD_CC1_RP_1A5_DIS();
	PD_CC2_RP_1A5_DIS();
#endif
#ifdef PD_CFG_RP_DEF
	PD_CC1_RP_DEF_DIS();
	PD_CC2_RP_DEF_DIS();
#endif
#ifdef PD_CFG_RD_CTRL
	PD_CC1_RD_DIS();
	PD_CC2_RD_DIS();
#endif

	// CC1
	uint8_t cc = TCPC_REG_ROLE_CTRL_CC1(role_ctrl_regval);
	if (cc == TYPEC_CC_RP) {
		switch (TCPC_REG_ROLE_CTRL_RP(role_ctrl_regval)) {
		case TYPEC_RP_3A0:
#ifdef PD_CFG_RP_3A0
			PD_CC1_RP_3A0_EN();
			break;	// This is NOT a bug
#endif				// TCPC will set the current advertisement to the highest available one
		case TYPEC_RP_1A5:
#ifdef PD_CFG_RP_1A5
			PD_CC1_RP_1A5_EN();
			break;
#endif
		default:
#ifdef PD_CFG_RP_DEF
			PD_CC1_RP_DEF_EN();
#endif
			break;
		}
	} else if (cc == TYPEC_CC_RD || cc == TYPEC_CC_RA) {
#ifdef PD_CFG_RD_CTRL
		PD_CC1_RD_EN();	// Rd and Ra can not coexist, remove Rd and install Ra if you need
#endif
	}

	// CC2
	cc = TCPC_REG_ROLE_CTRL_CC2(role_ctrl_regval);
	if (cc == TYPEC_CC_RP) {
		switch (TCPC_REG_ROLE_CTRL_RP(role_ctrl_regval)) {
		case TYPEC_RP_3A0:
#ifdef PD_CFG_RP_3A0
			PD_CC2_RP_3A0_EN();
			break;	// This is NOT a bug
#endif				// TCPC will set the current advertisement to the highest available one
		case TYPEC_RP_1A5:
#ifdef PD_CFG_RP_1A5
			PD_CC2_RP_1A5_EN();
			break;
#endif
		default:
#ifdef PD_CFG_RP_DEF
			PD_CC2_RP_DEF_EN();
#endif
			break;
		}
	}
	else if (cc == TYPEC_CC_RD || cc == TYPEC_CC_RA) {
#ifdef PD_CFG_RD_CTRL
		PD_CC2_RD_EN();	// Rd and Ra can not coexist, remove Rd and install Ra if you need
#endif
	}
}

void pd_set_vconn(uint8_t enabled, uint8_t orientation)
{
#ifdef PD_CFG_VCONN_CTRL
	if (!enabled) {
		// If ((POWER_CONTROL.EnableVconn=0)
		PD_CC1_VCONN_DIS();
		PD_CC2_VCONN_DIS();
	} else if (orientation){
		// If ((POWER_CONTROL.EnableVconn=1 and TCPC_CONTROL.PlugOrientation=1))
		// Apply Vconn to CC1, Monitor BMC on CC2
		PD_CC2_VCONN_DIS();
		PD_CC1_VCONN_EN();
	} else {
		// If ((POWER_CONTROL.EnableVconn=1 and TCPC_CONTROL.PlugOrientation=0))
		// Apply Vconn to CC2, Monitor BMC on CC1
		PD_CC1_VCONN_DIS();
		PD_CC2_VCONN_EN();
	}
#endif
}

/*
 * Sample and return the bus voltage in 100mV
 */
uint16_t pd_vbus_read_voltage(void)
{
#ifdef PD_CFG_VBUS_DECT
	uint32_t val = adc_read(PD_VBUS_SENSING_PIN);
	// mV = val / 4096 * 3.3 * 1000 / 10k * (10k+62k)
	// mV = val * 237600000 / 4096000 = val * 58.0078125
	val *= 2376;
	val >>= 12;	// 100mV
	return val;
#else
	return 0;
#endif
}
