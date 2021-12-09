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

void myGPIO_Init(void);
void myPA_Init(void);
void myTIM2_Init(void);
void myEXTI_Init(void);
void myLCD_Init(void);
void LcdWriteCmd(unsigned char Data,unsigned char command);
void LcdRegisterSelect(unsigned char Register);
void LcdWriteMode(void);
void LcdEnable(void);
void LcdDisable(void);
void LcdInitialize(void);
void LcdUpdate(float frequency);
static void ADC_Config(void);
static void DAC_Config(void);

// Declare/initialize your global variables here...
// NOTE: You'll need at least one global variable
// (say, timerTriggered = 0 or 1) to indicate
// whether TIM2 has started counting or not.

int timerTriggered = 0;

int
main(int argc, char* argv[])
{

	trace_printf("This is the project...\n");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIO_Init();		/* Initialize I/O port PA */
	myPA_Init();
	myTIM2_Init();		/* Initialize timer TIM2 */
	myEXTI_Init();		/* Initialize EXTI */
	myLCD_Init();
	LcdInitialize();
	ADC_Config();
	DAC_Config();
	ADC1->CR |= ((uint32_t)0b100);
	uint32_t voltage = ADC1->DR;
	DAC->DHR12R1 = voltage;
	while (1)
	{
		// Nothing is going on here...
		ADC1->CR |= ((uint32_t)0b100);
		uint32_t voltage = ADC1->DR;
		DAC->DHR12R1 = voltage;
	}

	return 0;

}

uint32_t ADCtoDAC(){
	uint32_t resistance =(uint32_t)((float)ADC1->DR *(float)(5000)/(float)0xFFF);
	return resistance;
}
void myGPIO_Init()
{
	/* Enable clock for port A and C */
	RCC->AHBENR |= ( RCC_AHBENR_GPIOCEN | RCC_AHBENR_GPIOAEN );

	GPIO_InitTypeDef IO;
	IO.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	IO.GPIO_Mode = GPIO_Mode_OUT;
	IO.GPIO_Speed = GPIO_Speed_50MHz;
	IO.GPIO_OType = GPIO_OType_PP;
	IO.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &IO);


	IO.GPIO_Pin = GPIO_Pin_0;
	IO.GPIO_Mode = GPIO_Mode_IN;
	IO.GPIO_Speed = GPIO_Speed_50MHz;
	IO.GPIO_OType = GPIO_OType_PP;
	IO.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &IO);


}

void myPA_Init(){

	RCC->AHBENR |= RCC_AHBPeriph_GPIOA;
	/* Configure PA2 as input */
	// Relevant register: GPIOA->MODER
	GPIOA->MODER &= ~(GPIO_MODER_MODER2);

	/* Ensure no pull-up/pull-down for PA2 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR2);
	//PA 1 as input 55 timer
	GPIOA->MODER &= ~(GPIO_MODER_MODER1);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR1);
}
void myTIM2_Init()
{
	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	// Relevant register: TIM2->CR1
	TIM2->CR1 = ((uint16_t)0x008C);

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;
	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	// Relevant register: TIM2->EGR
	TIM2->EGR = ((uint16_t)0x0001);

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
	NVIC->IP[3] = ((uint32_t)0x00FFFFFF);

	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(TIM2_IRQn);

	/* Enable update interrupt generation */
	// Relevant register: TIM2->DIER
	TIM2->DIER |= TIM_DIER_UIE;

}


void myEXTI_Init()
{
	EXTI->IMR |= ((uint32_t)0x00000001);	/* Unmask interrupts from EXTI0 line */
	EXTI->RTSR |= ((uint32_t)0x00000001);	/* EXTI0 line interrupts: set rising-edge trigger */

	/* Enable EXTI01 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI0_1_IRQn);

	/* Assign EXTI01 interrupt priority = 0 in NVIC */
	NVIC_SetPriority(EXTI0_1_IRQn, 0);

	/* Map EXTI1 line to PA1 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0] = 0x00000000;

	EXTI->IMR |= ((uint32_t)0x00000002);	/* Unmask interrupts from EXTI1 line */
	EXTI->RTSR |= ((uint32_t)0x00000002);	/* EXTI1 line interrupts: set rising-edge trigger */\

	NVIC->IP[1] = ((uint32_t)0xFF00FFFF);	//IRQ Prio for Exit1 - IRQ6

	NVIC->ISER[0] = ((uint32_t)0x00000040);	//IRQ Prio for Exit1 = IRQ6

	/* Map EXTI2 line to PA2 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0] = 0x00000000;


	//EXTI->IMR |= ((uint32_t)0x00000004);	/* Unmask interrupts from EXTI2 line */
	EXTI->RTSR |= ((uint32_t)0x00000004);	/* EXTI2 line interrupts: set rising-edge trigger */\

	NVIC->IP[1] = ((uint32_t)0xFF00FFFF);	//IRQ Prio for Exit2 - IRQ6

	NVIC->ISER[0] = ((uint32_t)0x00000040);	//IRQ Prio for Exit2 = IRQ6

}

void myLCD_Init(){
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	GPIO_InitTypeDef IO;
	IO.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_8
			| GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14
			 | GPIO_Pin_15;
	IO.GPIO_Mode = GPIO_Mode_OUT;
	IO.GPIO_Speed = GPIO_Speed_50MHz;
	IO.GPIO_OType = GPIO_OType_PP;
	IO.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &IO);

	IO.GPIO_Pin = GPIO_Pin_7;
	IO.GPIO_Mode = GPIO_Mode_IN;
	IO.GPIO_Speed = GPIO_Speed_50MHz;
	IO.GPIO_OType = GPIO_OType_PP;
	IO.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &IO);

	IO.GPIO_Pin = GPIO_Pin_0;
	IO.GPIO_Mode = GPIO_Mode_OUT;
	IO.GPIO_Speed = GPIO_Speed_50MHz;
	IO.GPIO_OType = GPIO_OType_PP;
	IO.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &IO);
}

void LcdWriteCmd(unsigned char Data,unsigned char command){
	GPIOB->ODR = ( Data << 8) | GPIO_Pin_0;	//Double Check
	LcdRegisterSelect(command);
	LcdWriteMode();
	LcdEnable();
	while( GPIO_ReadInputDataBit( GPIOB, GPIO_Pin_7) == Bit_RESET);	//Double Check
	LcdDisable();
	while( GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) == Bit_SET);	//Double Check

	GPIOB->ODR &= ~GPIO_Pin_0;
}

void LcdRegisterSelect(unsigned char Register){
	if(Register == 0){
		GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_RESET);

	}else{
		GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_SET);
	}
}

void LcdWriteMode(void){
	GPIO_WriteBit(GPIOB, GPIO_Pin_6, Bit_RESET);

}

void LcdEnable(void){
	GPIO_WriteBit(GPIOB, GPIO_Pin_4, Bit_SET);

}

void LcdDisable(void){
	GPIO_WriteBit(GPIOB, GPIO_Pin_4, Bit_RESET);

}

void LcdInitialize(void){
	LcdWriteCmd(0x28,0);
	LcdWriteCmd(0x0C,0);
	LcdWriteCmd(0x06,0);
	LcdWriteCmd(0x01,0);
	LcdWriteCmd(0x02,0);
	LcdWriteCmd(0x80,0);
}

static void ADC_Config(void){
	RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

	GPIOA->MODER &= 0xFFFFF3FF;
	GPIOA->MODER |= 0x00000C00;
	GPIOA->PUPDR &= 0xFFFFF3FF;
	GPIOA->PUPDR |= 0x0;

	ADC1->CFGR1 = ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD;

	ADC1->CHSELR |= ADC_CHSELR_CHSEL5;

	ADC1->SMPR |= ((uint32_t)0x00000007);

	ADC1->CR |= ((uint32_t)ADC_CR_ADEN);

	while(!(ADC1->ISR & ADC_CR_ADEN));

}

static void DAC_Config(void){

	RCC->APB1ENR |= RCC_APB1Periph_DAC;
	//RCC_APB1Periph_DAC

	GPIOA->MODER &= 0xFFFFFCFF;
	GPIOA->MODER |= 0x00000300;
	GPIOA->PUPDR &= 0xFFFFFCFF;
	GPIOA->PUPDR |= 0x00000000;

	DAC->CR &= 0xFFFFFFF9;
	DAC->CR |= 0x00000000;

	DAC->CR |= 0x00000001;
}

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM2->SR &= ~(TIM_SR_UIF);

		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM2->CR1 |= TIM_CR1_CEN;
	}
}

void EXTI0_1_IRQHandler(){
	//trace_printf("Exit 1 PR value: %i \n", EXTI->PR);	//Do not remove. It breaks. No it doesn't make sense.
	if ((EXTI->PR & EXTI_PR_PR1) != 0){
			uint16_t firstEdge = (TIM2->CR1 & TIM_CR1_CEN);
			uint32_t count;

				if(firstEdge == 0){
					//Clearing count register
					TIM2->CNT = (uint32_t)0x00;
					//Starting timer
					TIM2->CR1 |= TIM_CR1_CEN;
				}else{
					//Stop timer

					TIM2->CR1 &= ~TIM_CR1_CEN;
					//trace_printf("System clock: %u Hz\n", SystemCoreClock);
					count = TIM2->CNT;
					float frequency = (float)SystemCoreClock/(float)count;
					float period = 1/frequency;
					LcdUpdate(frequency);

				}
			//Clear EXTI1 interrupt pending flag
			EXTI->PR |= EXTI_PR_PR1;
		}else if((EXTI->PR & EXTI_PR_PR0) != 0 || (EXTI->IMR & EXTI_PR_PR1) == 0){
			if((EXTI->IMR & EXTI_PR_PR1) != 0){
			//	trace_printf("Mask works 1 \n");
				EXTI->IMR &= 0xFFFFFFFD;	//Masks Exit 1
				EXTI->IMR |= ((uint32_t)0x00000004);	/* Unmask interrupts from EXTI2 line */
			}else{
				//trace_printf("Mask works 2 \n");
				EXTI->IMR &= 0xFFFFFFFB;	//Masks Exit 2
				EXTI->IMR |= ((uint32_t)0x00000002);	/* Unmask interrupts from EXTI1 line */
			}

			//Clear EXTI0 interrupt pending flag
			EXTI->PR |= EXTI_PR_PR0;
	}


	//trace_printf("\nEXTI01 Triggered\n");
	//EXTI->PR |= 00000001;
}

void LcdUpdate(float frequency){
	char array[20];
	sprintf(array,"%f",frequency);
	LcdWriteCmd(0x02,0);
	LcdWriteCmd(0x46,1);
	LcdWriteCmd(0x3A,1);
	for(int i = 0; i< 9; i++){
		LcdWriteCmd((int)array[i], 1);
	}


	uint32_t resistance = ADCtoDAC();
	LcdWriteCmd(0xC0,0);
	char resist[20];
	sprintf(resist,"%d" ,(int)resistance);
	LcdWriteCmd(0x52,1);
	LcdWriteCmd(0x3A,1);
	if(resistance > 100){
		for(int i = 0; i< 4; i++){
			LcdWriteCmd((int)resist[i], 1);
		}
	}else{
		LcdWriteCmd(0x3C,1);
		LcdWriteCmd(0x31,1);
		LcdWriteCmd(0x30,1);
		LcdWriteCmd(0x30,1);
	}
}

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI2_3_IRQHandler()
{
//	trace_printf("Exit 2 PR value: %i \n", EXTI->PR);
	// Declare/initialize your local variables here...
	uint16_t firstEdge = (TIM2->CR1 & TIM_CR1_CEN);
	uint32_t count;

	/* Check if EXTI2 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR2) != 0)
	{
		if(firstEdge == 0){
			//Clearing count register
			TIM2->CNT = (uint32_t)0x00;
			//Starting timer
			TIM2->CR1 |= TIM_CR1_CEN;
		}else{
			//Stop timer
			TIM2->CR1 &= ~TIM_CR1_CEN;
			//trace_printf("System clock: %u Hz\n", SystemCoreClock);
			count = TIM2->CNT;
			float frequency = (float)SystemCoreClock/(float)count;
			float period = 1/frequency;
			//trace_printf("Period: %f seconds\n", period);
			//trace_printf("Exit 2 Frequency: %f Hz\n", frequency);
			LcdUpdate(frequency);

		}

		//Clear EXTI2 interrupt pending flag
		EXTI->PR |= EXTI_PR_PR2;
	}
}

//
	// 1. If this is the first edge:
	//	- Clear count register (TIM2->CNT).
	//	- Start timer (TIM2->CR1).
	//    Else (this is the second edge):
	//	- Stop timer (TIM2->CR1).
	//	- Read out count register (TIM2->CNT).
	//	- Calculate signal period and frequency.
	//	- Print calculated values to the console.
	//	  NOTE: Function trace_printf does not work
	//	  with floating-point numbers: you must use
	//	  "unsigned int" type to print your signal
	//	  period and frequency.
	//
	// 2. Clear EXTI2 interrupt pending flag (EXTI->PR).
	// NOTE: A pending register (PR) bit is cleared
	// by writing 1 to it.
	//
#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
