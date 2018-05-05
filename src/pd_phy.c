#include "platform.h"
#include "pd_phy.h"

#define DIV_ROUND_UP(x, y) (((x) + ((y) - 1)) / (y))
#define TX_CLOCK_DIV 48000000 / (2*300000) // Tx Clock = 2 x bit rate of BMC

uint32_t raw_samples_buf[PD_MAX_RAW_SIZE/4];
uint8_t* raw_samples;
uint16_t raw_ptr;	// Current Pos in Rx and number of raw bits in Tx
uint8_t* rx_ptr;
static uint8_t pd_selected_cc;

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
/* Sync-1      K-code       11000 Startsynch #1 */
/* Sync-2      K-code       10001 Startsynch #2 */
/* RST-1       K-code       00111 Hard Reset #1 */
/* RST-2       K-code       11001 Hard Reset #2 */
/* EOP         K-code       01101 EOP End Of Packet */
/* Reserved    Error        00000 */
/* Reserved    Error        00001 */
/* Reserved    Error        00010 */
/* Reserved    Error        00011 */
/* Reserved    Error        00100 */
/* Reserved    Error        00101 */
/* Reserved    Error        00110 */
/* Reserved    Error        01000 */
/* Reserved    Error        01100 */
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

void pd_select_cc(uint8_t cc) {
	GPIO_InitTypeDef GPIO_InitStruct;
	if (cc == PD_CC_1) {	// CC1 is CC, pull-up CC2, PB4 is Tx
		PD_CC_GPIO->ODR |= PD_CC2_PIN;
		GPIO_InitStruct.Pin = PD_CC2_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(PD_CC_GPIO, &GPIO_InitStruct);
		PD_CC_GPIO->ODR |= PD_CC2_PIN;

	    GPIO_InitStruct.Pin = GPIO_PIN_4;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
	    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	    pd_selected_cc = PD_CC_1;
	} else if (cc == PD_CC_2) {	// CC2 is CC, pull-up CC1, PA6 is Tx
		PD_CC_GPIO->ODR |= PD_CC1_PIN;
		GPIO_InitStruct.Pin = PD_CC1_PIN ;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(PD_CC_GPIO, &GPIO_InitStruct);
		PD_CC_GPIO->ODR |= PD_CC1_PIN;

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
	} else {
		PD_CC_GPIO->ODR &= ~(PD_CC1_PIN | PD_CC2_PIN);
		GPIO_InitStruct.Pin = PD_CC1_PIN | PD_CC2_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(PD_CC_GPIO, &GPIO_InitStruct);

		// PA6, PB4 to Hi-Z
		GPIO_InitStruct.Pin = GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_4;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		pd_selected_cc = PD_CC_NC;
	}
}

uint32_t pd_set_tx_pin(uint8_t cc) {
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
		    PD_CC_GPIO->MODER |= (3<<GPIO_MODER_MODER2_Pos); // PA4 -> Analog mode
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

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	pd_select_cc(0);
	GPIO_InitStruct.Pin = GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	GPIOB->ODR |= GPIO_PIN_11;

	// Comparator GPIO
	GPIO_InitStruct.Pin = PD_COMP_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(PD_COMP_GPIO, &GPIO_InitStruct);
	pd_rx_disable_monitoring();

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

	// TIM3
    __HAL_RCC_TIM3_CLK_ENABLE();
	/* --- set counter for RX timing : 12Mhz rate, free-running --- */
	TIM3->CR1 = 0x0000;
	TIM3->CR2 = 0x0000;
	TIM3->DIER = 0x0000;
	TIM3->ARR = 0xFFFF;	// Auto-reload value, 16-bit free running counter
	TIM3->PSC = (48000000 / 12000000) - 1; // Prescaler = fAPB1_Timer / 12MHz - 1;
	TIM3->CCR2 = (12000000 / 1000) * USB_PD_RX_TMOUT_US / 1000;	// Channel 2 - Timeout = 21600 Ticks

	// RM0091 page 449, IC4 = TI4FP4 input
	TIM3->CCMR2 = 0x100;
	// RM0091 page 450, CC4E=1, Capture enabled
	// CC4NP=1, CC4P=1, IC4 falling/rising
	TIM3->CCER = 0xB000;
	// RM0091 page 444, Enable update CC4 generation
	TIM3->EGR = 0x11;
	// RM0091 page 442, Clear flags
	TIM3->SR = 0;

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

	TIM14->CCMR1 = 0x68;	// 110: PWM mode 1, up counting. 0x08: enable reload
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

    pd_set_tx_pin(0);
}

void pd_rx_enable_monitoring(void) {
	/* clear comparator external interrupt */
	EXTI->PR = PD_COMP_PIN;		// Pending register
	/* enable comparator external interrupt */
	EXTI->IMR |= PD_COMP_PIN;
}

void pd_rx_disable_monitoring(void) {
	/* disable comparator external interrupt */
	EXTI->IMR &= ~ PD_COMP_PIN;
	/* clear comparator external interrupt */
	EXTI->PR = PD_COMP_PIN;
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
	DMA1_Channel3->CCR = 0;
}

void pd_rx_start() {GPIOB->ODR &= ~GPIO_PIN_11;
	raw_samples = (uint8_t*) raw_samples_buf;

	// Comparator GPIO -> Alternate function mode (for TIM3_CH4)
	GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = PD_COMP_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
    HAL_GPIO_Init(PD_COMP_GPIO, &GPIO_InitStruct);

	// Disable DMA
	DMA1_Channel3->CCR = 0;
	// Clear ISR
	DMA1->IFCR = 0xF00;

	// RM0091 page 441, Enable DMA request for IC4
	TIM3->DIER = 0x1000;

	// Priority very high, MSIZE=8, PSIZE=16, Memory increment mode
	DMA1_Channel3->CCR = (3<<DMA_CCR_PL_Pos) | (0<<DMA_CCR_MSIZE_Pos) | (1<<DMA_CCR_PSIZE_Pos) | DMA_CCR_MINC;
	DMA1_Channel3->CNDTR = PD_MAX_RAW_SIZE;
	DMA1_Channel3->CPAR = (uint32_t)(&(TIM3->CCR4));
	DMA1_Channel3->CMAR = (uint32_t)raw_samples;

	/* Flush data in write buffer so that DMA can get the lastest data */
	asm volatile("dsb;");

	// Enable DMA
	DMA1_Channel3->CCR |= DMA_CCR_EN;

	TIM3->CNT = 0;
	TIM3->EGR = 0x0001;	// UG = 1
	TIM3->SR = 0; // Clear flags
	TIM3->CR1 |= 1; // Enable Timer
}

void pd_rx_handler(void) {
	int next_idx;

	// See RM0091 page 214 & EXTI_PR
	if(__HAL_GPIO_EXTI_GET_IT(PD_COMP_PIN) != RESET) {
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
			return; // pd_rx_event(i);
		} else {
			/* do not trigger RX start, just clear int */
			__HAL_GPIO_EXTI_CLEAR_IT(PD_COMP_PIN);
		}
		rx_edge_ts_idx = next_idx;
	}
}

uint8_t pd_find_preamble(void) {
	raw_ptr = 2; 	// DMA1_CH3, TIM3_CH4

	uint32_t all = 0;
	raw_ptr = 2;
	while (raw_ptr < PD_BIT_LEN) {
		if ((PD_MAX_RAW_SIZE - DMA1_Channel3->CNDTR < raw_ptr + 1)) {
			while ((PD_MAX_RAW_SIZE - DMA1_Channel3->CNDTR < raw_ptr + 1)
					&& !(TIM3->SR & 4));
			if (TIM3->SR & 4) {
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
		raw_ptr++;

		all = (all >> 1) | (cnt <= PD_RX_THRESHOLD ? 1 << 31 : 0);

		if (all == 0xC7E3C78D) {	// SOP, Sync2 Sync1 Sync1 Sync1 101
			return PD_RX_SOP;
		} else if (all == 0xF33F3F3F) {
			return PD_RX_ERR_HARD_RESET; /* got HARD-RESET */
		} else if (all == 0x3c7fe0ff) {
			return PD_RX_ERR_CABLE_RESET; /* got CABLE-RESET */
		}
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

			0x36db6db6  should be SYNC-1
			0011 0110 1101 1011 0110 1101 1011 0110鈥�
			001  01 0 1 01  01  01 0 10 1  01  01 0

			0010101010101010101010
			| [Preamble---------->
			|
			Return value, end of preamble



			0xF33F3F3F  got HARD-RESET
			鈥�1111 0011 0011 1111 0011 1111 0011 1111鈥�
			1 1  001  001  1 1  001  1 1  001  1 1

			11001 00111 00111 00111
			RST-2 RST-1 RST-1 RST-1



			0x3c7fe0ff  got CABLE-RESET
			鈥�0011 1100 0111 1111 1110 0000 1111 1111鈥�
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

char pd_rx_decode_byte(void) {
	uint8_t nibble;
	uint8_t raw = 0;
	uint8_t cnt;

	for (uint8_t bit=0; bit<5; bit++) {
		if ((PD_MAX_RAW_SIZE - DMA1_Channel3->CNDTR < raw_ptr + 1)) {
			while ((PD_MAX_RAW_SIZE - DMA1_Channel3->CNDTR < raw_ptr + 1)
					&& !(TIM3->SR & 4));
			if (TIM3->SR & 4) {
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
			if ((PD_MAX_RAW_SIZE - DMA1_Channel3->CNDTR < raw_ptr + 1)) {
				while ((PD_MAX_RAW_SIZE - DMA1_Channel3->CNDTR < raw_ptr + 1)
						&& !(TIM3->SR & 4));
				if (TIM3->SR & 4) {
					return PD_RX_ERR_TIMEOUT;
				}
			}
			raw_ptr++; // Absorb next raw bit
		}
	}

	nibble = dec4b5b[raw&0x1F];
	if (nibble > 15)
		return nibble;

	raw = 0;
	for (uint8_t bit=5; bit<10; bit++) {
		if ((PD_MAX_RAW_SIZE - DMA1_Channel3->CNDTR < raw_ptr + 1)) {
			while ((PD_MAX_RAW_SIZE - DMA1_Channel3->CNDTR < raw_ptr + 1)
					&& !(TIM3->SR & 4));
			if (TIM3->SR & 4) {
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
			if ((PD_MAX_RAW_SIZE - DMA1_Channel3->CNDTR < raw_ptr + 1)) {
				while ((PD_MAX_RAW_SIZE - DMA1_Channel3->CNDTR < raw_ptr + 1)
						&& !(TIM3->SR & 4));
				if (TIM3->SR & 4) {
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
uint8_t ent=0;
int pd_rx_process(void) {
	ent++;
	uint16_t sop = pd_find_preamble();
	if (ent==2)
		while(1);
	if (sop != PD_RX_SOP)
		return sop;

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

	pd_rx_complete();
	if (crcr != crcc)
		return PD_RX_ERR_CRC;

	uint8_t id = PD_HEADER_ID(header);
	header = PD_HEADER(1, 0,	// Sink
			0, id, 0, PD_REV, 0);	// UFP

	pd_prepare_message(header, 0, 0);

	if (pd_tx() < 0)
		/* another packet recvd before we could send goodCRC */
		return PD_RX_ERR_INVAL;

	pd_rx_enable_monitoring();
	return 0;
}

char pd_tx(void) {
	pd_rx_disable_monitoring();

	if (pd_rx_started())
		return -1;

	// Config CC GPIO for Tx
	uint32_t cc = pd_set_tx_pin(pd_selected_cc);	// Get the GPIO mask

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
	DMA1_Channel3->CCR = 0;
	// Clear ISR
	DMA1->IFCR = 0xF00;
	// Priority very high, MSIZE=8, PSIZE=8, Memory increment mode, Memory->Peripheral
	DMA1_Channel3->CCR = (3<<DMA_CCR_PL_Pos) | (0<<DMA_CCR_MSIZE_Pos) | (0<<DMA_CCR_PSIZE_Pos) | DMA_CCR_MINC | DMA_CCR_DIR;
	DMA1_Channel3->CNDTR = DIV_ROUND_UP(raw_ptr, 8);
	DMA1_Channel3->CPAR = (uint32_t)(&(SPI1->DR));
	DMA1_Channel3->CMAR = (uint32_t)raw_samples_buf;

	// Flush data in write buffer so that DMA can get the lastest data
	asm volatile("dmb;");
	DMA1->IFCR = 0xF00;

	// Enable DMA
	DMA1_Channel3->CCR |= DMA_CCR_EN;

	SPI1->CR1 &= ~SPI_CR1_SSI;

	// Enable Tx Pin
	GPIOA->ODR &= ~cc;

	// Start PD Tx clock
	TIM14->CR1 |= TIM_CR1_CEN;

	while (SPI1->SR & SPI_SR_FTLVL)
		; /* wait for TX FIFO empty */
	while (SPI1->SR & SPI_SR_BSY)
		; /* wait for BSY == 0 */

	// Stop PD Tx clock
	TIM14->CR1 &= ~TIM_CR1_CEN;

	// Restore CC
	pd_set_tx_pin(0);

	__HAL_RCC_SPI1_FORCE_RESET();
	__HAL_RCC_SPI1_RELEASE_RESET();

	return 0;
}

static int b_toggle;
void pd_write_preamble(void) {
	uint32_t *msg = raw_samples_buf;

	/* 64-bit x2 preamble */
	msg[0] = 0xB4B4B4B4;	// Alternating bit sequence used for packet preamble
	msg[1] = 0xB4B4B4B4;	// 00 10 11 01 00 ..
	msg[2] = 0xB4B4B4B4;	// Starts with 0, ends with 1
	msg[3] = 0xB4B4B4B4;
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

void pd_prepare_message(uint16_t header, uint8_t cnt, const uint32_t *data) {
	pd_write_preamble();
	pd_write_sym(BMC(PD_SYNC1));
	pd_write_sym(BMC(PD_SYNC1));
	pd_write_sym(BMC(PD_SYNC1));
	pd_write_sym(BMC(PD_SYNC2));

	pd_write_sym(bmc4b5b[(header >> 0) & 0xF]);
	pd_write_sym(bmc4b5b[(header >> 4) & 0xF]);
	pd_write_sym(bmc4b5b[(header >> 8) & 0xF]);
	pd_write_sym(bmc4b5b[(header >> 12) & 0xF]);

	crc32_init();
	crc32_hash16(header);
	for (uint8_t i = 0; i < cnt; i++) {
		pd_write_sym(bmc4b5b[(data[i] >> 0) & 0xF]);
		pd_write_sym(bmc4b5b[(data[i] >> 4) & 0xF]);
		pd_write_sym(bmc4b5b[(data[i] >> 8) & 0xF]);
		pd_write_sym(bmc4b5b[(data[i] >> 12) & 0xF]);
		pd_write_sym(bmc4b5b[(data[i] >> 16) & 0xF]);
		pd_write_sym(bmc4b5b[(data[i] >> 20) & 0xF]);
		pd_write_sym(bmc4b5b[(data[i] >> 24) & 0xF]);
		pd_write_sym(bmc4b5b[(data[i] >> 28) & 0xF]);
		crc32_hash32(data[i]);
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

void pd_test_tx(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	pd_select_cc(PD_CC_1);

	while (GPIOB->IDR & GPIO_PIN_15);
	GPIOB->ODR &= ~GPIO_PIN_11;

	uint16_t header = PD_HEADER(1, 0,	// Sink
			0, 3, 0, 0, 0);	// UFP
	pd_prepare_message(header, 0, 0);

	pd_tx();

	while(1);
}
