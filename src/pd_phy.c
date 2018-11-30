#include "platform.h"
#include "pd_phy.h"
#include "pd.h"

// Configuration Start ---------------------------------------------------
#define PD_PHY_DETECT_ALL_SOP
// Rx GPIO Configuration ----------------------------------
#define PD_COMP_PIN1_ID 0 // PB0 as TIM3_CH3 during Rx
#define PD_COMP_PIN2_ID 1 // PB1 as TIM3_CH4 during Rx
#define PD_COMP_PIN1 (((uint16_t)0x0001U) << PD_COMP_PIN1_ID)
#define PD_COMP_PIN2 (((uint16_t)0x0001U) << PD_COMP_PIN2_ID)
#define PD_COMP_GPIO GPIOB
// Rx EXTI Configuration ----------------------------------
#define PD_COMP_EXTI EXTI0_1_IRQn
// Rx Timer Configuration ---------------------------------
// RM0091 page 449, IC4 = TI3FP4 input
#define PD_RX_CCMR_CC1() (TIM3->CCMR2 = 0x200)
// RM0091 page 449, IC4 = TI4FP4 input
#define PD_RX_CCMR_CC2() (TIM3->CCMR2 = 0x100)
// RM0091 page 450, CC4E=1, Capture enabled
// CC4NP=1, CC4P=1, IC4 falling/rising
#define PD_RX_CCER_EN() (TIM3->CCER = 0xB000)
#define PD_RX_CCER_DIS() (TIM3->CCER &= ~0xF000)
// RM0091 page 441, Enable DMA request for IC4
#define PD_RX_DIER_SET() (TIM3->DIER = 0x1000)
// Rx DMA Configuration -----------------------------------
#define PD_RX_DMA_SRCADDR (&(TIM3->CCR4))
#define PD_RX_DMA_CHANNEL DMA1_Channel3		// TIM3_CH4
// Configuration End -----------------------------------------------------

// Configurations that should not be changed -----------------------------
#define PD_RX_TIM_INCLK 48000000
#define PD_COMP_MASK (PD_COMP_PIN1 | PD_COMP_PIN2)
#define PD_COMP_PIN(cc) (cc==PD_CC_1 ? PD_COMP_PIN1 : PD_COMP_PIN2)
#define PD_RX_TIMEOUT() (TIM3->SR & 4)
#define PD_TX_DMA_CHANNEL DMA1_Channel3		// SPI1_Tx
#define PD_TX_DMA_IRQ DMA1_Channel2_3_IRQn
#define PD_TX_DMA_IRQ_CLEAR() (DMA1->IFCR |= DMA_IFCR_CGIF3)
// -----------------------------------------------------------------------

// Utility macros --------------------------------------------------------
#define DIV_ROUND_UP(x, y) (((x) + ((y) - 1)) / (y))
#define TX_CLOCK_DIV (PD_RX_TIM_INCLK / (2*300000)) // Tx Clock = 2 x bit rate of BMC

// Global variables ------------------------------------------------------
static uint32_t raw_samples_buf[PD_MAX_RAW_SIZE/4];
static uint8_t* raw_samples;
static uint16_t raw_ptr;	// Current Pos in Rx and number of raw bits in Tx
static uint8_t* rx_ptr;
static int b_toggle;	// Records bit sequences during Tx
static volatile uint8_t pd_selected_cc;
static volatile uint8_t rx_sop_type;
static volatile uint8_t* rx_bytes;
static volatile uint8_t tx_goingon;

static uint64_t rx_edge_ts[PD_RX_TRANSITION_COUNT];
static int rx_edge_ts_idx;

/* K-codes for special symbols */
#define PD_SYNC1 0x18
#define PD_SYNC2 0x11
#define PD_SYNC3 0x06
#define PD_RST1  0x07
#define PD_RST2  0x19
#define PD_EOP   0x0D

/* Encode 5 bits using Biphase Mark Coding */
#define BMC(x)   ((x &  1 ? 0x001 : 0x3FF) \
		^ (x &  2 ? 0x004 : 0x3FC) \
		^ (x &  4 ? 0x010 : 0x3F0) \
		^ (x &  8 ? 0x040 : 0x3C0) \
		^ (x & 16 ? 0x100 : 0x300))

#ifdef PD_PHY_DETECT_ALL_SOP
static const uint16_t bmcsop[] = {
	BMC(PD_SYNC1), BMC(PD_SYNC1), BMC(PD_SYNC1), BMC(PD_SYNC2),	// SOP
	BMC(PD_SYNC1), BMC(PD_SYNC1), BMC(PD_SYNC3), BMC(PD_SYNC3),	// SOPP
	BMC(PD_SYNC1), BMC(PD_SYNC3), BMC(PD_SYNC1), BMC(PD_SYNC3),	// SOPPP
	BMC(PD_SYNC1), BMC(PD_RST2), BMC(PD_RST2), BMC(PD_SYNC3),	// SOP_DBGP
	BMC(PD_SYNC1), BMC(PD_RST2), BMC(PD_SYNC3), BMC(PD_SYNC2)	// SOP_DBGPP
};
#endif	// #ifdef PD_PHY_DETECT_ALL_SOP

enum pd_rx_special_4b5b {
	TABLE_5b4b_SYNC1 = 16,
	TABLE_5b4b_SYNC2 = 17,
	TABLE_5b4b_RST1 = 18,
	TABLE_5b4b_RST2 = 19,
	TABLE_5b4b_EOP = 20,
	TABLE_5b4b_SYNC3 = 21,
	TABLE_5b4b_ERR = 22
};

/* 4b/5b + Bimark Phase encoding */
static const uint16_t bmc4b5b[] = {
/* 0 = 0000 */ BMC(0x1E) /* 11110 */,
/* 1 = 0001 */ BMC(0x09) /* 01001 */,
/* 2 = 0010 */ BMC(0x14) /* 10100 */,
/* 3 = 0011 */ BMC(0x15) /* 10101 */,
/* 4 = 0100 */ BMC(0x0A) /* 01010 */,
/* 5 = 0101 */ BMC(0x0B) /* 01011 */,
/* 6 = 0110 */ BMC(0x0E) /* 01110 */,
/* 7 = 0111 */ BMC(0x0F) /* 01111 */,
/* 8 = 1000 */ BMC(0x12) /* 10010 */,
/* 9 = 1001 */ BMC(0x13) /* 10011 */,
/* A = 1010 */ BMC(0x16) /* 10110 */,
/* B = 1011 */ BMC(0x17) /* 10111 */,
/* C = 1100 */ BMC(0x1A) /* 11010 */,
/* D = 1101 */ BMC(0x1B) /* 11011 */,
/* E = 1110 */ BMC(0x1C) /* 11100 */,
/* F = 1111 */ BMC(0x1D) /* 11101 */,
/* Sync-1   */ BMC(PD_SYNC1) /* 11000 Startsynch #1 */,
/* Sync-2   */ BMC(PD_SYNC2) /* 10001 Startsynch #2 */,
/* RST-1    */ BMC(PD_RST1) /* 00111 Reset #1 */,
/* RST-2    */ BMC(PD_RST2) /* 11001 Reset #2 */,
/* EOP      */ BMC(PD_EOP) /* 01101 EOP End Of Packet */,
/* Reserved    Error        00000 */
/* Reserved    Error        00001 */
/* Reserved    Error        00010 */
/* Reserved    Error        00011 */
/* Reserved    Error        00100 */
/* Reserved    Error        00101 */
/* Reserved    Error        00110 */
/* Reserved    Error        01000 */
/* Sync-2   */ BMC(PD_SYNC3) /* 01100 Startsynch #23 */
/* Reserved    Error        10000 */
/* Reserved    Error        11111 */
};

static const char dec4b5b[] = {
/* Error    */ TABLE_5b4b_ERR /* 00000 */,
/* Error    */ TABLE_5b4b_ERR /* 00001 */,
/* Error    */ TABLE_5b4b_ERR /* 00010 */,
/* Error    */ TABLE_5b4b_ERR /* 00011 */,
/* Error    */ TABLE_5b4b_ERR /* 00100 */,
/* Error    */ TABLE_5b4b_ERR /* 00101 */,
/* Error    */ TABLE_5b4b_SYNC3 /* 00110 K-code: Startsynch #3*/,
/* RST-1    */ TABLE_5b4b_RST1 /* 00111 K-code: Hard Reset #1 */,
/* Error    */ TABLE_5b4b_ERR /* 01000 */,
/* 1 = 0001 */ 0x01 /* 01001 */,
/* 4 = 0100 */ 0x04 /* 01010 */,
/* 5 = 0101 */ 0x05 /* 01011 */,
/* Error    */ TABLE_5b4b_ERR /* 01100 */,
/* EOP      */ TABLE_5b4b_EOP /* 01101 K-code: EOP End Of Packet */,
/* 6 = 0110 */ 0x06 /* 01110 */,
/* 7 = 0111 */ 0x07 /* 01111 */,
/* Error    */ TABLE_5b4b_ERR /* 10000 */,
/* Sync-2   */ TABLE_5b4b_SYNC2 /* 10001 K-code: Startsynch #2 */,
/* 8 = 1000 */ 0x08 /* 10010 */,
/* 9 = 1001 */ 0x09 /* 10011 */,
/* 2 = 0010 */ 0x02 /* 10100 */,
/* 3 = 0011 */ 0x03 /* 10101 */,
/* A = 1010 */ 0x0A /* 10110 */,
/* B = 1011 */ 0x0B /* 10111 */,
/* Sync-1   */ TABLE_5b4b_SYNC1 /* 11000 K-code: Startsynch #1 */,
/* RST-2    */ TABLE_5b4b_RST2 /* 11001 K-code: Hard Reset #2 */,
/* C = 1100 */ 0x0C /* 11010 */,
/* D = 1101 */ 0x0D /* 11011 */,
/* E = 1110 */ 0x0E /* 11100 */,
/* F = 1111 */ 0x0F /* 11101 */,
/* 0 = 0000 */ 0x00 /* 11110 */,
/* Error    */ TABLE_5b4b_ERR /* 11111 */,
};

/**
 * Set the active CC pin, as well as configure the comparator input pin and Tx pin
 */
void pd_select_cc(uint8_t cc) {
	GPIO_InitTypeDef GPIO_InitStruct;
	if (cc == PD_CC_1) {	// CC1 is CC, PB4 is Tx
	    GPIO_InitStruct.Pin = GPIO_PIN_4;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	    pd_selected_cc = PD_CC_1;

		// Comparator GPIO1
		PD_COMP_GPIO->MODER &=~ (0x03 << (PD_COMP_PIN1_ID<<1));	// Set to input
		PD_COMP_GPIO->PUPDR &=~ (0x03 << (PD_COMP_PIN1_ID<<1));	// Clear pull-up/pull-down
		PD_COMP_GPIO->PUPDR |= (0x01 << (PD_COMP_PIN1_ID<<1));	// Set to pull-up
		// EXTI0 to PB
		(*(SYSCFG->EXTICR + (PD_COMP_PIN1_ID >> 2))) &=~ ( (0x0FU) << ((PD_COMP_PIN1_ID&0x03U)<<2) );
		(*(SYSCFG->EXTICR + (PD_COMP_PIN1_ID >> 2))) |= ( (GPIO_GET_INDEX(PD_COMP_GPIO)) << ((PD_COMP_PIN1_ID&0x03U)<<2) );
		EXTI->EMR &=~ PD_COMP_PIN1;
		EXTI->IMR &=~ PD_COMP_PIN1;		// Disable the external interrupt
		EXTI->RTSR &=~ PD_COMP_PIN1;	// Disable rising trigger
		EXTI->FTSR |= PD_COMP_PIN1;		// Enable falling trigger

		PD_RX_CCER_DIS();
		PD_RX_CCMR_CC1();
		PD_RX_CCER_EN();
		// RM0091 page 444, Enable update CC4 generation
		TIM3->EGR = 0x11;
		// RM0091 page 442, Clear flags
		TIM3->SR = 0;
	} else if (cc == PD_CC_2) {	// CC2 is CC, PA6 is Tx
	    GPIO_InitStruct.Pin = GPIO_PIN_6;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
	    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	    // GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL6; // Clear AFSEL6, AF0_SPI
	    // GPIOA->MODER &= ~GPIO_MODER_MODER6;
	    // GPIOA->MODER |= (2<<GPIO_MODER_MODER6_Pos); // PA6 -> AF mode
	    // GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR6;
	    // GPIOA->OSPEEDR |= (3<<GPIO_OSPEEDR_OSPEEDR6_Pos); // PA6 -> High speed
	    // GPIOA->OTYPER &= ~GPIO_OTYPER_OT_6;	// PA6 -> Output Push-Pull
	    // GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR6;	// PA6 -> No pull-up/ pull-down

	    pd_selected_cc = PD_CC_2;

		// Comparator GPIO2
		PD_COMP_GPIO->MODER &=~ (0x03 << (PD_COMP_PIN2_ID<<1));	// Set to input
		PD_COMP_GPIO->PUPDR &=~ (0x03 << (PD_COMP_PIN2_ID<<1));	// Clear pull-up/pull-down
		PD_COMP_GPIO->PUPDR |= (0x01 << (PD_COMP_PIN2_ID<<1));	// Set to pull-up
		// EXTI0 to PB
		(*(SYSCFG->EXTICR + (PD_COMP_PIN2_ID >> 2))) &=~ ( (0x0FU) << ((PD_COMP_PIN2_ID&0x03U)<<2) );
		(*(SYSCFG->EXTICR + (PD_COMP_PIN2_ID >> 2))) |= ( (GPIO_GET_INDEX(PD_COMP_GPIO)) << ((PD_COMP_PIN2_ID&0x03U)<<2) );
		EXTI->EMR &=~ PD_COMP_PIN2;
		EXTI->IMR &=~ PD_COMP_PIN2;		// Disable the external interrupt
		EXTI->RTSR &=~ PD_COMP_PIN2;	// Disable rising trigger
		EXTI->FTSR |= PD_COMP_PIN2;		// Enable falling trigger

		PD_RX_CCER_DIS();
		PD_RX_CCMR_CC2();
		PD_RX_CCER_EN();
		// RM0091 page 444, Enable update CC4 generation
		TIM3->EGR = 0x11;
		// RM0091 page 442, Clear flags
		TIM3->SR = 0;
	} else {
		GPIO_InitStruct.Pin = PD_CC1_PIN | PD_CC2_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(PD_CC_GPIO, &GPIO_InitStruct);

		// PA6, PB4 to Hi-Z
		GPIO_InitStruct.Pin = GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_4;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		pd_selected_cc = PD_CC_NC;

		// Disable Comparator IT
		GPIO_InitStruct.Pin = PD_COMP_MASK;
		HAL_GPIO_Init(PD_COMP_GPIO, &GPIO_InitStruct);
	}
}

/**
 * Get ready for Tx, change active cc to open drain and return the bit mask (to be used for pulling down the GPIO)
 */
static uint32_t pd_tx_prepare_cc(uint8_t cc) {
	static uint8_t last_cc;

	if (cc == PD_CC_1) {	// PD_CC_1=0, PB4=Tx
		last_cc = PD_CC_1;

		PD_CC_GPIO->ODR |= GPIO_PIN_2;
		PD_CC_GPIO->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR2;
		PD_CC_GPIO->OSPEEDR |= (3<<GPIO_OSPEEDR_OSPEEDR2_Pos); // PA2 -> High speed
	    PD_CC_GPIO->OTYPER |= GPIO_PIN_2;	// PA2 -> Open drain
	    PD_CC_GPIO->PUPDR &= ~GPIO_PUPDR_PUPDR2;	// PA2 -> No pull-up/ pull-down

	    PD_CC_GPIO->MODER &= ~GPIO_MODER_MODER2;
	    PD_CC_GPIO->MODER |= (1<<GPIO_MODER_MODER2_Pos); // PA2 -> General purpose output mode

		return PD_CC1_PIN;
	} else if (cc == PD_CC_2) {	// PD_CC_2=0, PA6=Tx
		last_cc = PD_CC_2;

		PD_CC_GPIO->ODR |= GPIO_PIN_4;
		PD_CC_GPIO->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR4;
		PD_CC_GPIO->OSPEEDR |= (3<<GPIO_OSPEEDR_OSPEEDR4_Pos); // PA4 -> High speed
	    PD_CC_GPIO->OTYPER |= GPIO_PIN_4;	// PA4 -> Open drain
	    PD_CC_GPIO->PUPDR &= ~GPIO_PUPDR_PUPDR4;	// PA4 -> No pull-up/ pull-down

	    PD_CC_GPIO->MODER &= ~GPIO_MODER_MODER4;
	    PD_CC_GPIO->MODER |= (1<<GPIO_MODER_MODER4_Pos); // PA4 -> General purpose output mode

	    return PD_CC2_PIN;
	} else {
	    if (last_cc == PD_CC_1) {
	    	last_cc = 0;

			PD_CC_GPIO->ODR |= GPIO_PIN_2;
		    PD_CC_GPIO->MODER &= ~GPIO_MODER_MODER2;
		    PD_CC_GPIO->MODER |= (3<<GPIO_MODER_MODER2_Pos); // PA2 -> Analog mode
	    } else if (last_cc == PD_CC_2) {
	    	last_cc = 0;

			PD_CC_GPIO->ODR |= GPIO_PIN_4;
		    PD_CC_GPIO->MODER &= ~GPIO_MODER_MODER4;
		    PD_CC_GPIO->MODER |= (3<<GPIO_MODER_MODER4_Pos); // PA4 -> Analog mode
	    }
	}

	return 0;
}

void pd_init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;

	// Enable SYSCFG for EXTI
	__HAL_RCC_SYSCFG_CLK_ENABLE();

	rx_sop_type = PD_RX_IDLE;
	pd_select_cc(PD_CC_NC);

	pd_rx_disable_monitoring();

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(PD_COMP_EXTI, 1, 0);
	HAL_NVIC_EnableIRQ(PD_COMP_EXTI);
	HAL_NVIC_SetPriority(PD_TX_DMA_IRQ, 1, 0);
	HAL_NVIC_EnableIRQ(PD_TX_DMA_IRQ);

	// TIM3
    __HAL_RCC_TIM3_CLK_ENABLE();
	/* --- set counter for RX timing : 12Mhz rate, free-running --- */
	TIM3->CR1 = 0x0000;
	TIM3->CR2 = 0x0000;
	TIM3->DIER = 0x0000;
	TIM3->ARR = 0xFFFF;	// Auto-reload value, 16-bit free running counter
	TIM3->PSC = (PD_RX_TIM_INCLK / 12000000) - 1; // Prescaler = fAPB1_Timer / 12MHz - 1;
	TIM3->CCR2 = (12000000 / 1000) * USB_PD_RX_TMOUT_US / 1000;	// Channel 2 - Timeout = 21600 Ticks

	// DMA
	__HAL_RCC_DMA1_CLK_ENABLE();

	// Tx Timer (Supplies clock to Half-duplex SPI)
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM14;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	__HAL_RCC_TIM14_CLK_ENABLE();
	TIM14->CR1 = 0x0000;
	TIM14->DIER = 0x0000;
	TIM14->ARR = TX_CLOCK_DIV;	// Auto-reload value : 600000 Khz overflow
	TIM14->PSC = 0;
	TIM14->CCR1 = TIM14->ARR / 2; // 50% duty cycle

	TIM14->CCMR1 = 0x68;	// 110: PWM mode 1, up counting. 0x08: enable preload
	TIM14->CCER = 1; 		// CH1 output enable
	TIM14->EGR = 1;			// UG=1, Reinitialize the counter and generates an update of the registers

	// Tx SPI
	__HAL_RCC_SPI1_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    pd_tx_prepare_cc(0);
}

void pd_rx_enable_monitoring(void) {
	/* clear comparator external interrupt */
	EXTI->PR = PD_COMP_MASK;		// Pending register
	/* enable comparator external interrupt */
	EXTI->IMR |= PD_COMP_MASK;
}

void pd_rx_disable_monitoring(void) {
	/* disable comparator external interrupt */
	EXTI->IMR &= ~ PD_COMP_MASK;
	/* clear comparator external interrupt */
	EXTI->PR = PD_COMP_MASK;
}

uint32_t pd_rx_started(void) {
	/* is the sampling timer running ? */
	return TIM3->CR1 & 1;	// RM0091 436, CEN == 1 ??
}

void pd_rx_complete() {
	/* stop stampling TIM2 */
	TIM3->CR1 &= ~1; // CEN = 0;
	TIM3->DIER = 0; // Disable any possible interrupt / DMA request
	// Disable DMA
	PD_RX_DMA_CHANNEL->CCR = 0;
}

void pd_rx_start() {
	raw_samples = (uint8_t*) raw_samples_buf;
	rx_bytes = (uint8_t*) raw_samples_buf - 32;

	// Comparator GPIO -> Alternate function mode (for TIM3_CH4)
	GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = PD_COMP_PIN(pd_selected_cc);
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
    HAL_GPIO_Init(PD_COMP_GPIO, &GPIO_InitStruct);

	// Disable DMA
	PD_RX_DMA_CHANNEL->CCR = 0;
	// Clear ISR
	DMA1->IFCR = 0xF00;

	// Enable Rx timer's DMA request
	PD_RX_DIER_SET();

	// Priority very high, MSIZE=8, PSIZE=16, Memory increment mode
	PD_RX_DMA_CHANNEL->CCR = (3<<DMA_CCR_PL_Pos) | (0<<DMA_CCR_MSIZE_Pos) | (1<<DMA_CCR_PSIZE_Pos) | DMA_CCR_MINC;
	PD_RX_DMA_CHANNEL->CNDTR = PD_MAX_RAW_SIZE;
	PD_RX_DMA_CHANNEL->CPAR = (uint32_t)PD_RX_DMA_SRCADDR;
	PD_RX_DMA_CHANNEL->CMAR = (uint32_t)raw_samples;

	/* Flush data in write buffer so that DMA can get the lastest data */
	asm volatile("dsb;");

	// Enable DMA
	PD_RX_DMA_CHANNEL->CCR |= DMA_CCR_EN;

	TIM3->CNT = 0;
	TIM3->EGR = 0x0001;	// UG = 1
	TIM3->SR = 0; // Clear flags
	TIM3->CR1 |= 1; // Enable Timer
}

static inline uint8_t pd_find_preamble(void) {
	raw_ptr = 2; 	// DMA1_CH3, TIM3_CH4

	uint32_t all = 0;
	raw_ptr = 2;
	while (raw_ptr < PD_BIT_LEN) {
		if ((PD_MAX_RAW_SIZE - PD_RX_DMA_CHANNEL->CNDTR < raw_ptr + 1)) {
			while ((PD_MAX_RAW_SIZE - PD_RX_DMA_CHANNEL->CNDTR < raw_ptr + 1)
					&& !PD_RX_TIMEOUT());
			if (PD_RX_TIMEOUT()) {
				return PD_RX_ERR_TIMEOUT;
			}
		}

		uint8_t cnt;
		if (raw_samples[raw_ptr] > raw_samples[raw_ptr-1]) {
			cnt = raw_samples[raw_ptr] - raw_samples[raw_ptr-1];
		} else {
			cnt = 255 - raw_samples[raw_ptr-1];
			cnt += 1 + raw_samples[raw_ptr];
		}

		all = (all >> 1) | (cnt <= PD_RX_THRESHOLD ? 1 << 31 : 0);

#ifdef PD_PHY_DETECT_ALL_SOP
		if (all == 0x36DB6DB6) {
			raw_ptr--;
#else
		if (all == 0xC7E3C78D) {
			raw_ptr++;
#endif
			return PD_RX_SOP;				// Potential SOP* packet
		} else if (all == 0xF33F3F3F) {
			return PD_RX_ERR_HARD_RESET;	// Got Hard-Reset
		} else if (all == 0x3c7fe0ff) {
			return PD_RX_ERR_CABLE_RESET;	// Got Cable-Reset
		}

		raw_ptr++;
	}

		/*
		Explanation to the magic numbers:
		1. The timer and comparator setup determines that, raw_samples[] contains number which is proportional to the width of pulses
		2. "all = (all >> 1) | (cnt <= PERIOD_THRESHOLD ? 1 << 31 : 0);" gives 1 for narrow pulses and 0 for wide pulses
		3. According to BMC encoding (refer to USD PD specification),
			a data 1 is a transition in the middle of a frame
			while a data 0 doesn't transit from L->H or H-L during the frame.
		4. So the decoding rule becomes, read "all" as a 32-bit binary number,
			a 0 corresponds to actual data bit 0 and two adjacent 1 in "all" correspond to actual data bit 1.

			0xC7E3C78D got SOP
			1100 0111 1110 0011 1100 0111 1000 1101
			1 00 01 1  1 0 001  1 00 01 1  000 1 01

			10001 11000 11000 11000 101
			Sync2 Sync1 Sync1 Sync1 |[Preamble---------->
			|
			Return value, end of preamble & SOP

			0x36DB6DB6 got potential SOP
			‭0011 0110 1101 1011 0110 1101 1011 0110
			001  01 0 1 01  01  01 0 1 01  01  01 0
			|
			bit0 of Sync-1‬

			0xF33F3F3F got HARD-RESET
			1111 0011 0011 1111 0011 1111 0011 1111
			1 1  001  001  1 1  001  1 1  001  1 1

			11001 00111 00111 00111
			RST-2 RST-1 RST-1 RST-1



			0x3c7fe0ff got CABLE-RESET
			0011 1100 0111 1111 1110 0000 1111 1111
			001  1 00 01 1  1 1  1 0 0000 1 1  1 1
			00110 00111 11000 00111 1
			Sync3 RST-1 Sync1 RST-1 | Last bit of preamble

			See also:
			USB PD specification 3.0:
			Table 5-1 4b5b Symbol Encoding Table
			Table 5-5 SOP ordered set
			Table 5-11 Hard Reset ordered set
			Table 5-12 Cable Reset ordered set
		*/

	return PD_RX_ERR_INVAL;
}

/*
 * Attempt to decode a byte from incoming BMC traffic
 * Return:
 * 		0 - decoded a data byte and placed into the buffer(raw_ptr), raw_ptr is automatically increased
 * 		PD_RX_ERR_TIMEOUT - Timeout, Error
 * 		Other: K-code e.g. Sync-1 or EOP, or Error(TABLE_5b4b_ERR)
 */
static inline uint8_t pd_rx_decode_byte(void) {
	uint8_t nibble;
	uint8_t raw = 0;
	uint8_t cnt;

	for (uint8_t bit=0; bit<5; bit++) {
		if ((PD_MAX_RAW_SIZE - PD_RX_DMA_CHANNEL->CNDTR < raw_ptr + 1)) {
			while ((PD_MAX_RAW_SIZE - PD_RX_DMA_CHANNEL->CNDTR < raw_ptr + 1)
					&& !PD_RX_TIMEOUT());
			if (PD_RX_TIMEOUT()) {
				return PD_RX_ERR_TIMEOUT;
			}
		}
		if (raw_samples[raw_ptr] > raw_samples[raw_ptr-1]) {
			cnt = raw_samples[raw_ptr] - raw_samples[raw_ptr-1];
		} else {
			cnt = 255 - raw_samples[raw_ptr-1];
			cnt += 1 + raw_samples[raw_ptr];
		}
		raw_ptr++;
		if (cnt <= PD_RX_THRESHOLD) {
			raw |= 1 << bit;
			if ((PD_MAX_RAW_SIZE - PD_RX_DMA_CHANNEL->CNDTR < raw_ptr + 1)) {
				while ((PD_MAX_RAW_SIZE - PD_RX_DMA_CHANNEL->CNDTR < raw_ptr + 1)
						&& !PD_RX_TIMEOUT());
				if (PD_RX_TIMEOUT()) {
					return PD_RX_ERR_TIMEOUT;
				}
			}
			raw_ptr++; // Absorb next raw bit
		}
	}

	nibble = dec4b5b[raw&0x1F];
	if (nibble > 15)	// Not a data byte
		return nibble;

	raw = 0;
	for (uint8_t bit=5; bit<10; bit++) {
		if ((PD_MAX_RAW_SIZE - PD_RX_DMA_CHANNEL->CNDTR < raw_ptr + 1)) {
			while ((PD_MAX_RAW_SIZE - PD_RX_DMA_CHANNEL->CNDTR < raw_ptr + 1)
					&& !PD_RX_TIMEOUT());
			if (PD_RX_TIMEOUT()) {
				return PD_RX_ERR_TIMEOUT;
			}
		}
		if (raw_samples[raw_ptr] > raw_samples[raw_ptr-1]) {
			cnt = raw_samples[raw_ptr] - raw_samples[raw_ptr-1];
		} else {
			cnt = 255 - raw_samples[raw_ptr-1];
			cnt += 1 + raw_samples[raw_ptr];
		}
		raw_ptr++;
		if (cnt <= PD_RX_THRESHOLD) {
			raw |= 1 << (bit-5);
			if ((PD_MAX_RAW_SIZE - PD_RX_DMA_CHANNEL->CNDTR < raw_ptr + 1)) {
				while ((PD_MAX_RAW_SIZE - PD_RX_DMA_CHANNEL->CNDTR < raw_ptr + 1)
						&& !PD_RX_TIMEOUT());
				if (PD_RX_TIMEOUT()) {
					return PD_RX_ERR_TIMEOUT;
				}
			}
			raw_ptr++; // Absorb next raw bit
		}
	}

	raw = dec4b5b[raw&0x1F];
	if (raw > 15)
		return raw;

	rx_ptr++;
	*rx_ptr = (raw<<4) | nibble;

	return 0;
}

static inline uint8_t pd_rx_process(void) {
	uint8_t sop = pd_find_preamble();
	if (sop == PD_RX_SOP) {
#ifdef PD_PHY_DETECT_ALL_SOP
		uint8_t kcode1 = pd_rx_decode_byte();
		uint8_t kcode2 = pd_rx_decode_byte();
		uint8_t kcode3 = pd_rx_decode_byte();
		uint8_t kcode4 = pd_rx_decode_byte();
		if (kcode1 == TABLE_5b4b_SYNC1 && kcode2 == TABLE_5b4b_SYNC1
			&& kcode3 == TABLE_5b4b_SYNC1 && kcode4 == TABLE_5b4b_SYNC2){
			sop = PD_RX_SOP;
		} else if (kcode1 == TABLE_5b4b_SYNC1 && kcode2 == TABLE_5b4b_SYNC1
				&& kcode3 == TABLE_5b4b_SYNC3 && kcode4 == TABLE_5b4b_SYNC3){
			sop = PD_RX_SOPP;
		} else if (kcode1 == TABLE_5b4b_SYNC1 && kcode2 == TABLE_5b4b_SYNC3
				&& kcode3 == TABLE_5b4b_SYNC1 && kcode4 == TABLE_5b4b_SYNC3){
			sop = PD_RX_SOPPP;
		} else if (kcode1 == TABLE_5b4b_SYNC1 && kcode2 == TABLE_5b4b_RST2
				&& kcode3 == TABLE_5b4b_RST2 && kcode4 == TABLE_5b4b_SYNC3){
			sop = PD_RX_SOP_DBGP;
		} else if (kcode1 == TABLE_5b4b_SYNC1 && kcode2 == TABLE_5b4b_RST2
				&& kcode3 == TABLE_5b4b_SYNC3 && kcode4 == TABLE_5b4b_SYNC2){
			sop = PD_RX_SOP_DBGPP;
		} else {
			return PD_RX_ERR_UNSUPPORTED_SOP;
		}
#endif
	} else if (sop ==PD_RX_ERR_HARD_RESET) {
		return PD_RX_ERR_HARD_RESET;
	}
#ifdef PD_PHY_DETECT_ALL_SOP
	else if (sop == PD_RX_ERR_CABLE_RESET) {
		return PD_RX_ERR_CABLE_RESET;
	}
#endif

	// Decode the header
	rx_ptr = raw_samples - 1;
	if (pd_rx_decode_byte() > 15)
		return PD_RX_ERR_INVAL;
	if (pd_rx_decode_byte() > 15)
		return PD_RX_ERR_INVAL;

	crc32_init();
	uint16_t header = *((uint16_t*)raw_samples);
	uint8_t cnt = PD_HEADER_CNT(header);

	crc32_hash16(header);

	/* read payload data */
	for (uint8_t p = 0; p < cnt; p++) {
		if (pd_rx_decode_byte() > 15)
			return PD_RX_ERR_INVAL;
		crc32_hash8(raw_samples[2+4*p]);
		if (pd_rx_decode_byte() > 15)
			return PD_RX_ERR_INVAL;
		crc32_hash8(raw_samples[3+4*p]);
		if (pd_rx_decode_byte() > 15)
			return PD_RX_ERR_INVAL;
		crc32_hash8(raw_samples[4+4*p]);
		if (pd_rx_decode_byte() > 15)
			return PD_RX_ERR_INVAL;
		crc32_hash8(raw_samples[5+4*p]);
	}

	uint32_t crcc = crc32_result();

	if (pd_rx_decode_byte() > 15)
		return PD_RX_ERR_INVAL;
	uint32_t crcr = (*rx_ptr);
	if (pd_rx_decode_byte() > 15)
		return PD_RX_ERR_INVAL;
	crcr |= (*rx_ptr)<<8;
	if (pd_rx_decode_byte() > 15)
		return PD_RX_ERR_INVAL;
	crcr |= (*rx_ptr)<<16;
	if (pd_rx_decode_byte() > 15)
		return PD_RX_ERR_INVAL;
	crcr |= (*rx_ptr)<<24;

	if (crcr != crcc)
		return PD_RX_ERR_CRC;

	return sop;
}

static inline void copy_msg(void) {
	uint8_t byte_count = 2 + (PD_HEADER_CNT(*((uint16_t*)raw_samples)) << 2);

	for (uint8_t i=0; i<byte_count; i++){
		rx_bytes[i] = raw_samples[i];
	}
}

void pd_rx_isr_handler(void) {
	int next_idx;
	uint32_t pd_comp_pin = PD_COMP_PIN(pd_selected_cc);

	// See RM0091 page 214 & EXTI_PR
	if(__HAL_GPIO_EXTI_GET_IT(pd_comp_pin) != RESET) {
		rx_edge_ts[rx_edge_ts_idx] = timestamp_get();
		next_idx = (rx_edge_ts_idx == PD_RX_TRANSITION_COUNT - 1) ?
					0 : rx_edge_ts_idx + 1;

		/*
		 * If we have seen enough edges in a certain amount of
		 * time, then trigger RX start.
		 */
		if ((rx_edge_ts[rx_edge_ts_idx] -
		     rx_edge_ts[next_idx])
		     < PD_RX_TRANSITION_WINDOW) {

			/*
			 * ignore the comparator IRQ until we are done
			 * with current message
			 */
			pd_rx_disable_monitoring();

			/* start sampling */
			pd_rx_start();

			/* trigger the analysis in the task */
			rx_sop_type = pd_rx_process();

			pd_rx_complete();	// Ignore EOP

			uint16_t header = *((uint16_t*)raw_samples);
			if (PD_HEADER_CNT(header) == 0 && PD_HEADER_TYPE(header) == PD_CTRL_GOOD_CRC) {
				// Received GoodCRC message

				// Copy received data to buffer
				copy_msg();
				rx_sop_type += PD_RX_SOP_GOODCRC;
			} else {
				// Received other GoodCRC
				header = tcpc_phy_get_goodcrc_header(
						rx_sop_type, 				// SOP type
						PD_HEADER_ID(header)		// Message ID
						);
				if (header == 0xFE) {
					// The Rx buffer is full, do not send GoodCRC
					rx_sop_type = PD_RX_ERR_OVERRUN;
				} else if (header != 0xFF) {
					// Respond GoodCRC if necessary

					// Copy received data to buffer
					copy_msg();

					pd_prepare_message(rx_sop_type, 2, (uint8_t*)(&header));
					pd_tx(1);
				}
			}

			// End of ISR handler
		} else {
			/* do not trigger RX start, just clear int */
			EXTI->PR = pd_comp_pin;
		}
		rx_edge_ts_idx = next_idx;
	}
}

uint8_t pd_phy_get_rx_type(void) {
	return rx_sop_type;
}

void pd_phy_clear_rx_type(void) {
	rx_sop_type = PD_RX_IDLE;
}

uint16_t pd_phy_get_rx_msg(uint8_t* payload) {
	uint16_t header = *((uint16_t*)rx_bytes);
	if (payload == 0)
		return header;

	uint8_t byte_count = (PD_HEADER_CNT(header) << 2);

	for (uint8_t i=0; i<byte_count; i++){
		payload[i] = rx_bytes[i+2];
	}

	return header;
}

uint8_t pd_phy_is_txing(void) {
	return tx_goingon;
}

/**
 * Transmit an encoded PD message to the active cc line. Return negative if something goes wrong.
 * send_goodcrc should be 1 when send GoodCRC in respond to incoming message, in this case,
 * rx monitoring is not enabled immediately. send_goodcrc should be 0 so that rx monitoring is
 * resumed immediately after message transmission.
 */
char pd_tx(uint8_t send_goodcrc) {
	pd_rx_disable_monitoring();

	if (pd_rx_started() || tx_goingon)
		return -1;

	// 0xFF - Monitor for incoming GoodCRC after transmission
	tx_goingon = send_goodcrc ? 1 : 0xFF;

	// Config CC GPIO for Tx
	uint32_t cc = pd_tx_prepare_cc(pd_selected_cc);	// Get the GPIO mask

	// Initialize SPI ----------------------------------------------------------------
	SPI1->CR2 = SPI_CR2_TXDMAEN | (7<<SPI_CR2_DS_Pos); // Enable DMA, 8 data bit
	// Enable the slave SPI: LSB first, force NSS, TX only, CPHA
	// When the SSM bit is set, the NSS pin input is replaced with the value from the SSI bit.
	// BIDIMODE: 1: 1-line bidirectional data mode selected
	// BIDIOE: 1: Output enabled (transmit-only mode) otherwise receive-only mode
	// CPHA: 1: The second clock transition is the first data capture edge
	SPI1->CR1 =	SPI_CR1_SPE | SPI_CR1_LSBFIRST
			 | SPI_CR1_SSM | SPI_CR1_BIDIMODE
			 | SPI_CR1_BIDIOE | SPI_CR1_CPHA | SPI_CR1_SSI ;

	/*
	 * Set timer to one tick before reset so that the first tick causes
	 * a rising edge on the output.
	 */
	TIM14->CNT = TX_CLOCK_DIV - 1;

	// Prepare DMA -------------------------------------------------------------------
	// Disable DMA
	PD_TX_DMA_CHANNEL->CCR = 0;
	// Clear ISR
	DMA1->IFCR = 0xF00;

	PD_TX_DMA_CHANNEL->CCR =
			(3<<DMA_CCR_PL_Pos) |		// Priority very high
			(0<<DMA_CCR_MSIZE_Pos) |	// MSIZE=8
			(0<<DMA_CCR_PSIZE_Pos) |	// PSIZE=8
			DMA_CCR_MINC |				// Memory increment mode
			DMA_CCR_DIR |				// Memory->Peripheral
			DMA_CCR_TCIE;				// Generate an interrupt at the end of DMA transfer
	PD_TX_DMA_CHANNEL->CNDTR = DIV_ROUND_UP(raw_ptr, 8);
	PD_TX_DMA_CHANNEL->CPAR = (uint32_t)(&(SPI1->DR));
	PD_TX_DMA_CHANNEL->CMAR = (uint32_t)raw_samples_buf;

	// Flush data in write buffer so that DMA can get the latest data
	asm volatile("dsb;");
	DMA1->IFCR = 0xF00;

	// Enable DMA
	PD_TX_DMA_CHANNEL->CCR |= DMA_CCR_EN;

	SPI1->CR1 &= ~SPI_CR1_SSI;

	// Enable Tx Pin
	GPIOA->ODR &= ~cc;

	// Start PD Tx clock
	TIM14->CR1 |= TIM_CR1_CEN;

	// After Tx done, DMA_TransferComplete interrupt will be triggered
	return 0;
}

void pd_tx_isr_handler(void) {
	PD_TX_DMA_IRQ_CLEAR();

	// Wait for the end of transmission
	while (SPI1->SR & SPI_SR_FTLVL)
		; /* wait for TX FIFO empty */
	while (SPI1->SR & SPI_SR_BSY)
		; /* wait for BSY == 0 */

	// Stop PD Tx clock
	TIM14->CR1 &= ~TIM_CR1_CEN;

	// Restore CC
	pd_tx_prepare_cc(PD_CC_NC);

	__HAL_RCC_SPI1_FORCE_RESET();
	__HAL_RCC_SPI1_RELEASE_RESET();

	__asm__ __volatile__("cpsid i");
	if (tx_goingon == 0xFF)
		pd_rx_enable_monitoring();

	tx_goingon = 0;
	__asm__ __volatile__("cpsie i");
}

void pd_write_preamble(void) {
	uint32_t *msg = raw_samples_buf;

	/* 64-bit x2 preamble */
	msg[0] = 0xB4B4B4B4;	// Alternating bit sequence used for packet preamble
	msg[1] = msg[0];		// 00 10 11 01 00 ..
	msg[2] = msg[0];		// Starts with 0, ends with 1
	msg[3] = msg[0];
	b_toggle = 0x3FF; /* preamble ends with 1 */
	raw_ptr = 2*64;
}

void pd_write_sym(uint32_t val10) {
	uint32_t *msg = raw_samples_buf;
	int word_idx = raw_ptr / 32;
	int bit_idx = raw_ptr % 32;
	uint32_t val = b_toggle ^ val10;
	b_toggle = val & 0x200 ? 0x3FF : 0;
	if (bit_idx <= 22) {
		if (bit_idx == 0)
			msg[word_idx] = 0;
		msg[word_idx] |= val << bit_idx;
	} else {
		msg[word_idx] |= val << bit_idx;
		msg[word_idx+1] = val >> (32 - bit_idx);
		/* side effect: clear the new word when starting it */
	}
	raw_ptr += 5*2;
}

void pd_write_last_edge(void) {
	uint32_t *msg = raw_samples_buf;
	int word_idx = raw_ptr / 32;
	int bit_idx = raw_ptr % 32;

	if (bit_idx == 0)
		msg[word_idx] = 0;

	if (!b_toggle /* last bit was 0 */) {
		/* transition to 1, another 1, then 0 */
		if (bit_idx == 31) {
			msg[word_idx++] |= 1 << bit_idx;
			msg[word_idx] = 1;
		} else {
			msg[word_idx] |= 3 << bit_idx;
		}
	}
	/* ensure that the trailer is 0 */
	msg[word_idx+1] = 0;

	raw_ptr += 3;
}

/**
 * Encode message bytes into BMC and 4b5b form
 * header: the 16-bit header
 * cnt: the number of 32-bit payloads (same as the Number of Data Objects field in the Message Header)
 * data: a pointer to the payload buffer, 0 if the message is a control message
 */
void pd_prepare_message(uint8_t sop_type, uint8_t cnt, const uint8_t* data) {
	uint8_t i;

	pd_write_preamble();

#ifdef PD_PHY_DETECT_ALL_SOP
	// Write SOP
	for (i = 0; i < 4; i++)
		pd_write_sym(bmcsop[i+(sop_type<<2)]);
#else
	pd_write_sym(bmc4b5b[TABLE_5b4b_SYNC1]);
	pd_write_sym(bmc4b5b[TABLE_5b4b_SYNC1]);
	pd_write_sym(bmc4b5b[TABLE_5b4b_SYNC1]);
	pd_write_sym(bmc4b5b[TABLE_5b4b_SYNC2]);
#endif


	crc32_init();
	for (i = 0; i < cnt; i++) {
		pd_write_sym(bmc4b5b[data[i] & 0xF]);
		pd_write_sym(bmc4b5b[(data[i] >> 4) & 0xF]);
		crc32_hash8(data[i]);
	}

	uint32_t crc = crc32_result();
	pd_write_sym(bmc4b5b[(crc >> 0) & 0xF]);
	pd_write_sym(bmc4b5b[(crc >> 4) & 0xF]);
	pd_write_sym(bmc4b5b[(crc >> 8) & 0xF]);
	pd_write_sym(bmc4b5b[(crc >> 12) & 0xF]);
	pd_write_sym(bmc4b5b[(crc >> 16) & 0xF]);
	pd_write_sym(bmc4b5b[(crc >> 20) & 0xF]);
	pd_write_sym(bmc4b5b[(crc >> 24) & 0xF]);
	pd_write_sym(bmc4b5b[(crc >> 28) & 0xF]);

	pd_write_sym(BMC(PD_EOP));

	pd_write_last_edge();
}

void pd_prepare_reset(uint8_t hard_rest)
{
	pd_write_preamble();

	if (hard_rest) {
		/* Hard-Reset: 3x RST-1 + 1x RST-2 */
		pd_write_sym(bmc4b5b[TABLE_5b4b_RST1]);
		pd_write_sym(bmc4b5b[TABLE_5b4b_RST1]);
		pd_write_sym(bmc4b5b[TABLE_5b4b_RST1]);
		pd_write_sym(bmc4b5b[TABLE_5b4b_RST2]);
	} else {
		/* Cable-Reset: RST-1, Sync-1, RST-1, Sync-3 */
		pd_write_sym(bmc4b5b[TABLE_5b4b_RST1]);
		pd_write_sym(bmc4b5b[TABLE_5b4b_SYNC1]);
		pd_write_sym(bmc4b5b[TABLE_5b4b_RST1]);
		pd_write_sym(bmc4b5b[TABLE_5b4b_SYNC3]);
	}
	/* Ensure that we have a final edge */
	pd_write_last_edge();
}
