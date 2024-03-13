/*
 * button.h
 *
 *  Created on: Jul 18, 2016
 *      Author: Eric Middleton
 *
 * @edit: Phillip Jones 05/30/2019 : Removed uneeded helper functions
 */

#ifndef BUTTON_H_
#define BUTTON_H_

#include <stdint.h>
#include <stdbool.h>
#include <inc/tm4c123gh6pm.h>
//#include <driverlib/interrupt.h>

//extern volatile uint8_t BUTTON1_STATE;
//extern volatile uint8_t BUTTON2_STATE;
//extern volatile uint8_t BUTTON3_STATE;
//extern volatile uint8_t BUTTON4_STATE;

//initialize the push buttons
void button_init();
void BUTTON_Handler(void);


///Non-blocking call
///Returns highest value button being pressed, 0 if no button pressed
uint8_t button_getButton();


#endif /* BUTTON_H_ */
