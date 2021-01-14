//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.
// Course: ECE 355 "Microprocessor-Based Systems".
// This is template code for Part 2 of Introductory Lab.
//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"

// ----------------------------------------------------------------------------
//
// STM32F0 empty sample (trace via $(trace)).
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)

void myGPIOA_Init(void);
void myGPIOB_Init(void);
void myGPIOC_Init(void);
void myTIM2_Init(void);
void myTIM3_Init(void);
void myEXTI_Init(void);
void myDAC_Init(void);
void myADC_Init(void);
void LCD_Init(void);
void LCD_command(uint32_t cmd);
void LCD_update_upper(void);
void LCD_update_lower(void);



// Declare/initialize your global variables here...
// NOTE: You'll need at least one global variable
// (say, timerTriggered = 0 or 1) to indicate
// whether TIM2 has started counting or not.

//Min voltage = 0.06 - 0.07
//Max voltage = 2.18 - 2.23
// Resitance ~70 to 5000

volatile int first_edge = 1; // 1 for first edge, 0 otherwise
float resistance = 0;
float frequency = 0;
char digits[] = {'0','1','2','3','4','5','6','7','8', '9'};


int
main(int argc, char* argv[])
{

	trace_printf("This is the main lab assignment\n");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);
	myGPIOB_Init();		/* Initialize I/O port PB */
	LCD_Init();			/* Initialize LCD */
	myGPIOA_Init();		/* Initialize I/O port PA */
	myGPIOC_Init();		/* Initialize I/O port PC */
	myTIM2_Init();		/* Initialize timer TIM2 */
	myTIM3_Init(); 		/* Initialize timer TIM3 */
	myEXTI_Init();		/* Initialize EXTI */
	myDAC_Init();		/* Initialize DAC */
	myADC_Init();		/* Initialize ADC */



	ADC1->CR |= ADC_CR_ADSTART;							/* Start ADC, only once cause of continuous conversion */

while (1)
	{
		while((ADC1->ISR & ADC_ISR_EOC) == 0 );			/* Wait for end of conversion */
		uint32_t ADC_value = ADC1->DR;					/* Read Value from ADC data register */
		DAC->DHR12R1 = ADC_value;						/* Send ADC value to DAC register */
		resistance = (float)5000*(ADC_value)/4095;		/* Convert ADC value to resistance */
	}
	return 0;
}

/* Initialize PA1 as input and PA4 as output */
void myGPIOA_Init()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;			/* Enable clock for GPIOA peripheral */
	GPIOA->MODER &= ~(GPIO_MODER_MODER1_1);		/* Configure PA1 as input */
	GPIOA->MODER &= ~(GPIO_MODER_MODER4_0);		/* Configure PA4 as analog output 	 */
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);		/* Ensure no pull-up/pull-down for PA1 */
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR4);		/* Ensure no pull-up/pull-down for PA4 */
}

/* Initialize PB7 as input, rest of PB is output */
void myGPIOB_Init()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;			/* Enable clock for GPIOB peripheral */
	GPIOB->MODER &= ~(GPIO_MODER_MODER7_1);		/* Configure PB7 as input */

	/* Configure PB4-PB6, and PB8-PB15 as output*/
	GPIOB->MODER |= (GPIO_MODER_MODER4_0 |
			GPIO_MODER_MODER5_0 |
			GPIO_MODER_MODER6_0 |
			GPIO_MODER_MODER8_0 |
			GPIO_MODER_MODER9_0 |
			GPIO_MODER_MODER10_0 |
			GPIO_MODER_MODER11_0 |
			GPIO_MODER_MODER12_0 |
			GPIO_MODER_MODER13_0 |
			GPIO_MODER_MODER14_0 |
			GPIO_MODER_MODER15_0);

	/* Ensure no pull-up/pull-down for PB4- PB15 */
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR4 |
			GPIO_PUPDR_PUPDR5 |
			GPIO_PUPDR_PUPDR6 |
			GPIO_PUPDR_PUPDR7 |
			GPIO_PUPDR_PUPDR8 |
			GPIO_PUPDR_PUPDR9 |
			GPIO_PUPDR_PUPDR10 |
			GPIO_PUPDR_PUPDR11 |
			GPIO_PUPDR_PUPDR12 |
			GPIO_PUPDR_PUPDR13 |
			GPIO_PUPDR_PUPDR14 |
			GPIO_PUPDR_PUPDR15);

}

/* Initialize PC1 as input */
void myGPIOC_Init()
{
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;		/* Enable clock for GPIOC peripheral */
	GPIOC->MODER &= ~(GPIO_MODER_MODER1_1);	/* Configure PC1 as analog input */
	GPIOC->PUPDR &= ~(GPIO_PUPDR_PUPDR1);	/* Ensure no pull-up/pull-down for PC1 */
}

/* Initialize TIM2 for frequency calculation based on 555 timer */
void myTIM2_Init()
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;		/* Enable clock for TIM2 peripheral */
	TIM2->CR1 = ((uint16_t)0x008C);			/* Configure TIM2: buffer auto-reload, count up, stop on overflow,enable update events, interrupt on overflow only */
	TIM2->PSC = myTIM2_PRESCALER;			/* Set clock prescaler value (No prescaling) */
	TIM2->ARR = myTIM2_PERIOD;				/* Set auto-reloaded delay (Max 0xFFFFFFFF)*/
	TIM2->EGR = ((uint16_t)0x0001);			/* Update timer registers (Re initialize TiIM2 and generate update of registers) */
	NVIC_SetPriority(TIM2_IRQn, 0);			/* Assign TIM2 interrupt priority = 0 (highest) in NVIC */
	NVIC_EnableIRQ(TIM2_IRQn);				/* Enable TIM2 interrupts in NVIC */
	TIM2->DIER |= TIM_DIER_UIE;				/* Enable update interrupt generation */
}

/* Initialize timer 3 for LCD refresh rate at 25Hz */
void myTIM3_Init()
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;		/* Enable clock for TIM2 peripheral */
	TIM3->CR1 = ((uint16_t)0x008C);			/* Configure TIM3: buffer auto-reload, count up, stop on overflow, enable update events, interrupt on overflow only */
	TIM3->PSC = (uint32_t)0X0000BB80;		/* Set clock prescaler value, 1ms per decimal value*/
	TIM3->ARR = (uint32_t)0x00000027;		/* Set auto-reloaded delay of 40ms */
	TIM3->EGR = ((uint16_t)0x0001);			/* Update timer registers (Re initialize TIM3 and generate update of registers) */
	NVIC_SetPriority(TIM3_IRQn, 64);		/* Assign TIM3 interrupt priority = 64 (2nd highest) in NVIC. LCD refresh less important than calculation interrupts */
	NVIC_EnableIRQ(TIM3_IRQn);				/* Enable TIM3 interrupts in NVIC */
	TIM3->DIER |= TIM_DIER_UIE;				/* Enable update interrupt generation */
	TIM3->CR1 = ((uint16_t)0x0001);			/* Start Timer */

}

/* Initialize rising edge trigger using PA1 and EXTI1 */
void myEXTI_Init()
{
	SYSCFG->EXTICR[0] = ((uint16_t)0x0080);		/* Map EXTI1 line to PA1 */
	EXTI->RTSR = EXTI_RTSR_TR1;					/* EXTI1 line interrupts: set rising-edge trigger 0x00000002*/
	EXTI->IMR = EXTI_IMR_MR1;					/* Unmask interrupts from EXTI1 line 0x00000002*/
	NVIC_SetPriority(EXTI0_1_IRQn, 0);			/* Assign EXTI1 interrupt priority = 0 in NVIC */
	NVIC_EnableIRQ(EXTI0_1_IRQn);				/* Enable EXTI1 interrupts in NVIC */
}

/* Initialize DAC */
void myDAC_Init()
{
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;		/* Enable Clock for DAC */
	DAC->CR |= DAC_CR_EN1;					/* Enable DAC out 1 channel */
}

/* Initialize ADC in continuous mode */
void myADC_Init()
{
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;			/* Enable Clock for DAC */
	ADC1->CFGR1 |= ADC_CFGR1_CONT;				/* Continuous Configuration */
	ADC1->SMPR |= ADC_SMPR_SMP; 				/* Set sample rate to 111, 239 ADC clock cycles */
	ADC1->CHSELR |= ADC_CHSELR_CHSEL11;			/* PC1 uses channel 11 	*/
	ADC1->CR |= ADC_CR_ADEN;					/* Enable ADC1*/
	while((ADC1->ISR & ADC_ISR_ADRDY) == 0);	/* Wait for ADC to be ready */
}


/* Initialize LCD for our display requirements */
void LCD_Init()
{
	LCD_command(0x00003800);	// Set LCD for 2 lines of 8 chars, and DDRAM access using 8-bit interface
	LCD_command(0x00000C00);	// Display On, no cursor, no blinking
	LCD_command(0x00000600);	// Auto Increment DDRAM address, not shifted
	LCD_command(0x00000100);	// Clear display
}

/* Function to send command to LCD DDRAM using handshaking */
void LCD_command(uint32_t cmd)
{
	GPIOB->BRR = 0xFFFF;
	GPIOB->BSRR = cmd;							//Send Command
	GPIOB->BSRR |= 0x00000010;					//Handshake enable
	while ((GPIOB->IDR & GPIO_IDR_7) == 0);		//Handshake Done
	GPIOB->BRR  |= 0x0010;					//De-assert enable
	while ((GPIOB->IDR & GPIO_IDR_7) != 0);		//wait for Done to de-assert

}

/* Funtion to update the upper 8 chars on the LCD to reflect frequency */
void LCD_update_upper()
{
	/* Breakdown frequency measurement to each digit */
	int freq1 = ((int)frequency % 10);
	int freq2 = (((int)frequency/10) % 10);
	int freq3 = (((int)frequency/100) % 10);
	int freq4 = (((int)frequency/1000));

	LCD_command(0x00008000); /* DDRAM address for LCD top row */

	/*Send first line data */
	LCD_command(0x00004620); /* F */
	LCD_command(0x00003A20); /* : */

	/* Four digits from Frequency, most significant to least significant */
	LCD_command(0x0000 << 24 | digits[freq4] << 8 | 0x0020);
	LCD_command(0x0000 << 24 | digits[freq3] << 8 | 0x0020);
	LCD_command(0x0000 << 24 | digits[freq2] << 8 | 0x0020);
	LCD_command(0x0000 << 24 | digits[freq1] << 8 | 0x0020);

	LCD_command(0x00004820); /* H */
	LCD_command(0x00007A20); /* z */

}

void LCD_update_lower()
{
	/* Breakdown resistance measurement to each digit */
	int res1 = ((int)resistance % 10);
	int res2 = (((int)resistance/10) % 10);
	int res3 = (((int)resistance/100) % 10);
	int res4 = (((int)resistance/1000));


	LCD_command(0x0000C000); /* DDRAM address for LCD bottom row */

	/*Send first line data */
	LCD_command(0x00005220); /* R */
	LCD_command(0x00003A20); /* : */

	/* Four digits from Resistance, most significant to least significant */
	LCD_command(0x0000 << 24 | digits[res4] << 8 | 0x0020);
	LCD_command(0x0000 << 24 | digits[res3] << 8 | 0x0020);
	LCD_command(0x0000 << 24 | digits[res2] << 8 | 0x0020);
	LCD_command(0x0000 << 24 | digits[res1] << 8 | 0x0020);

	LCD_command(0x00004F20); /* O */
	LCD_command(0x00006820); /* h */
}

/* TIM2 interrupt due to overflow */
void TIM2_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");
		TIM2->SR = ((uint16_t)0x0000);		/* Clear update interrupt flag */
		TIM2->CR1 |= ((uint16_t)0x0001);	/* Restart stopped timer */
	}
}

/* TIM3 interrupt for LCD refresh */
void TIM3_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM3->SR & TIM_SR_UIF) != 0)
		{
			LCD_update_upper();					/* Update upper 8 chars of LCD */
			LCD_update_lower();					/* Update lower 8 chars of LCD */
			TIM3->SR = ((uint16_t)0x0000);		/* Clear interrupt flag */
			TIM3->CR1 |= ((uint16_t)0x0001);	/* Restart stopped timer */
		}
}

/* Rising Edge interrupt and Frequency calculation */
void EXTI0_1_IRQHandler()
{

	float cycles_elapsed = 0;	/* Variable to store timer count of clock cycles */

	/* Check if EXTI2 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0)
	{
		if (first_edge){
			first_edge = 0;						/*Clear first edge flag*/
			TIM2->CNT = ((uint16_t)0x0000);		/*Clear count register*/
			TIM2->CR1 = ((uint16_t)0x0001);		/*Start Timer*/
		} else {
			cycles_elapsed = TIM2->CNT;						/*Get cycles since last edge*/
			TIM2->CR1 = ((uint16_t)0x0000);					/*Stop Timer*/
			frequency = SystemCoreClock/cycles_elapsed;		/* Calculate Frequency*/
			first_edge = 1;									/* Reset first edge flag */
		}
		EXTI->PR = EXTI_PR_PR1;					/* Clear EXTI1 interrupt pending flag */
	}
}


#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
