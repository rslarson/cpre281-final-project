/*
 * button.c
 *
 *  Created on: Jul 18, 2016
 *      Author: Eric Middleton, Zhao Zhang, Chad Nelson, & Zachary Glanz.
 *
 *  @edit: Lindsey Sleeth and Sam Stifter on 02/04/2019
 *  @edit: Phillip Jones 05/30/2019: Merged Spring 2019 version with Fall 2018
 *  @edit: Diane Rover 02/01/20: Corrected comments about ordering of switches for new LCD board and added busy-wait on PRGPIO
 */



//The buttons are on PORTE 3:0
// GPIO_PORTE_DATA_R -- Name of the memory mapped register for GPIO Port E,
// which is connected to the push buttons
#include <stdio.h>
#include <stdlib.h>

#include "button.h"
//#include "Timer.H"


//char *button_msg;

//volatile uint8_t BUTTON1_STATE = 0;
//volatile uint8_t BUTTON2_STATE = 0;
//volatile uint8_t BUTTON3_STATE = 0;
//volatile uint8_t BUTTON4_STATE = 0;

static bool initialized = false;


/**
 * Initialize PORTE and configure bits 0-3 to be used as inputs for the buttons.
 */
void button_init() {
    if (initialized) {
        return;
    }
    //button_msg = calloc(50, sizeof(char));

	// 1) Turn on PORTE system clock, do not modify other clock enables
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4; //0b 01 0000

	// You may need to add a delay here of several clock cycles for the clock to start, e.g., execute a simple dummy assignment statement, such as "long delay = SYSCTL_RCGCGPIO_R".

  // Instead, use the PRGPIO register and busy-wait on the peripheral ready bit for PORTE.
	while ((SYSCTL_PRGPIO_R & 0x10) == 0) {
	    long delay = SYSCTL_RCGCGPIO_R;
	};
	// 2) Set the buttons as inputs, do not modify other PORTE wires
	GPIO_PORTE_DIR_R &= ~(0x0F);// 0b1111 0000

	// 3) Enable digital functionality for button inputs,
	//    do not modify other PORTE enables
	GPIO_PORTE_DEN_R |= 0x0F; //0b0000 1111

	// Unmask unused pins
//	GPIO_PORTE_IM_R &= 0x0F;
//
//	//GPIO_PORTE_PUR_R |= 0x0F;
//
//	// Configure GPIO Interrupt Sense to Edges
//	GPIO_PORTE_IS_R  &= 0xF0;
//	GPIO_PORTE_IEV_R |= 0x0F;
//	GPIO_PORTE_IBE_R |= 0x0F;
//
//	// Clear any existing interrupts
//	GPIO_PORTE_MIS_R &= 0xF0;
//
//	// Configure Interrupt to be handled by NVIC controller
//	GPIO_PORTE_IM_R |= 0x0F;
//
//	// Enable interrupt handling on button bush
//	NVIC_PRI1_R &= ~(0xF0);
//	NVIC_PRI1_R |= 0x40;
//	NVIC_EN0_R |= 0x10;
//
//	IntRegister(INT_GPIOE, BUTTON_Handler);
//	IntMasterEnable();

	initialized = true;
}


//void BUTTON_Handler(void) {
//    if (GPIO_PORTE_MIS_R & 0x01) {
//        if(BUTTON1_STATE) BUTTON1_STATE = 0;
//        else BUTTON1_STATE = 1;
//        sprintf(button_msg, "Button1: %d\n\r", BUTTON1_STATE);
//    }
//    else if (GPIO_PORTE_MIS_R & 0x02) {
//        if(BUTTON1_STATE) BUTTON1_STATE = 0;
//        else BUTTON2_STATE = 1;
//        sprintf(button_msg, "Button2: %d\n\r", BUTTON2_STATE);
//    }
//    else if (GPIO_PORTE_MIS_R & 0x04) {
//        if(BUTTON1_STATE) BUTTON1_STATE = 0;
//        else BUTTON3_STATE = 1;
//        sprintf(button_msg, "Button3: %d\n\r", BUTTON3_STATE);
//    }
//    else if (GPIO_PORTE_MIS_R & 0x08) {
//        if(BUTTON1_STATE) BUTTON1_STATE = 0;
//        else BUTTON4_STATE = 1;
//        sprintf(button_msg, "Button4: %d\n\r", BUTTON4_STATE);
//
//    }
//    uart_sendStr(button_msg);
//    GPIO_PORTE_ICR_R |= 0x0F;
//    return;
//}

/**
 * Returns the position of the rightmost button being pushed.
 * @return the position of the rightmost button being pushed. 1 is the leftmost button, 4 is the rightmost button.  0 indicates no button being pressed
 */
uint8_t button_getButton() {
    uint8_t out = 0;

    if(!(GPIO_PORTE_DATA_R & 0x08)){
       out |= 0x08;
    }
    if (!(GPIO_PORTE_DATA_R & 0x04)){
        out |= 0x04;
    }
    if (!(GPIO_PORTE_DATA_R & 0x02)){
        out |= 0x02;
    }
    if (!(GPIO_PORTE_DATA_R & 0x01)){
        out |= 0x01;
    }
    return out;
}
    
