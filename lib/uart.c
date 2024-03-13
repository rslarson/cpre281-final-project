/*
*
*   uart.c
*
*
*
*   @author
*   @date
*/

#include <inc/tm4c123gh6pm.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "uart.h"

static bool uart_initialized = false;

void uart_init(void) {
    if (uart_initialized) {
        return;
    }

    //enable clock to GPIO port B
    SYSCTL_RCGCGPIO_R |= 0x02;

    //enable clock to UART1
    SYSCTL_RCGCUART_R |= 0x02;

    //wait for GPIOB and UART1 peripherals to be ready
    while ((SYSCTL_PRGPIO_R & 0x02) == 0) {};
    while ((SYSCTL_PRUART_R & 0x02) == 0) {};

    //enable alternate functions on port B pins
    GPIO_PORTB_AFSEL_R |= 0x03;

    //enable digital functionality on port B pins
    GPIO_PORTB_DEN_R |= 0x03;

    //enable UART1 Rx and Tx on port B pins
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & 0xFFFFFF00) | 0x00000011;

    //calculate baud rate
    float BRD = 16000000 / (16.0 * 115200);
    uint16_t iBRD = (int)BRD;
    uint16_t fBRD = (int)(((BRD - iBRD) * 64) + 0.5);

    //turn off UART1 while setting it up
    UART1_CTL_R |= 0 << 0;

    //set baud rate
    //note: to take effect, there must be a write to LCRH after these assignments
    UART1_IBRD_R = iBRD;
    UART1_FBRD_R = fBRD;

    //set frame, 8 data bits, 1 stop bit, no parity, no FIFO
    //note: this write to LCRH must be after the BRD assignments
    UART1_LCRH_R = 0x60;

    //use system clock as source
    //note from the datasheet UARTCCC register description:
    //field is 0 (system clock) by default on reset
    //Good to be explicit in your code
    UART1_CC_R &= 0x0;

    //re-enable UART1 and also enable RX, TX (three bits)
    //note from the datasheet UARTCTL register description:
    //RX and TX are enabled by default on reset
    //Good to be explicit in your code
    //Be careful to not clear RX and TX enable bits
    //(either preserve if already set or set them)
    UART1_CTL_R |= (1UL << 0 | 1UL << 8 | 1UL << 9);

    uart_initialized = true;
}

void uart_sendChar(char data){
	while(UART1_FR_R & UART_FR_TXFF){
	    if(UART1_FR_R == 0){
	        break;
	    }
	}
	UART1_DR_R = data;
}

char uart_receive(void){
    uint32_t ret;
	char rdata = 0;
	while(UART1_FR_R & UART_FR_RXFE){
	    if(UART1_FR_R == 0 ){
	        break;
	    }
	}

	ret = UART1_DR_R;
	if(ret & 0xF00){
	    GPIO_PORTB_DATA_R = 0xF;
	}else {
	    rdata = (char) (UART1_DR_R & 0xFF);
	}
	return rdata;
}

void uart_sendStr(const char *data){
	while (*data != '\0') {
	    uart_sendChar(*data);
	    data++;
	}
}
