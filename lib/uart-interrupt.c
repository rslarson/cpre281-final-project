/*
*
*   uart-interrupt.c
*
*
*
*   @author
*   @date
*/

#include <inc/tm4c123gh6pm.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "Timer.h"

#include "uart-interrupt.h"
#include "robot.h"

#define BIT0 0x01
#define BIT1 0x02
#define BIT2 0x04
#define BIT3 0x08
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80

// These variables are declared as examples for your use in the interrupt handler.
volatile char command_byte = '\0'; // byte value for special character used as a command
volatile int command_flag = 0; // flag to tell the main program a special command was received

volatile command_t UART_Command = NO_COMMAND;
volatile char* UART_Byte_Stream = NULL;
volatile char* inputByteStream = NULL;
volatile uint8_t *outputByteStream = NULL;
bool UART_Payload_Recieved = false;

static bool uart_initialized = false;

void uart_init(void){
    if (uart_initialized) {
        return;
    }
    UART_Byte_Stream = calloc(8, sizeof(char));
    inputByteStream = calloc(16, sizeof(char));
    outputByteStream = calloc(16, sizeof(char));


    //enable clock to GPIO port B
    SYSCTL_RCGCGPIO_R |= 0x02;

    //enable clock to UART1
    SYSCTL_RCGCUART_R |= 0x02;

    //wait for GPIOB and UART1 peripherals to be ready
    while ((SYSCTL_PRGPIO_R & BIT1) == 0) {};
    while ((SYSCTL_PRUART_R & BIT1) == 0) {};

    //enable digital functionality on port B pins
    GPIO_PORTB_DEN_R |= (BIT0 | BIT1);

    //enable alternate functions on port B pins
    GPIO_PORTB_AFSEL_R |= (BIT0 | BIT1);

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
    UART1_LCRH_R = (UART_LCRH_FEN | UART_LCRH_WLEN_8);

    //use system clock as source
    //note from the datasheet UARTCCC register description:
    //field is 0 (system clock) by default on reset
    //Good to be explicit in your code
    UART1_CC_R = 0x0;

    //////Enable interrupts

    //first clear RX interrupt flag (clear by writing 1 to ICR)
    UART1_ICR_R |= BIT4;

    //Set UART Interrupt FIFO Level to 1/4
     UART1_IFLS_R = 0x10;

    //enable RX raw interrupts in interrupt mask register
    UART1_IM_R |= UART_IM_RXIM;

    //NVIC setup: set priority of UART1 interrupt to 1 in bits 21-23
    NVIC_PRI1_R = (NVIC_PRI1_R & 0xFF0FFFFF) | 0x00200000;

    //NVIC setup: enable interrupt for UART1, IRQ #6, set bit 6
    NVIC_EN0_R |= BIT6;

    //tell CPU to use ISR handler for UART1 (see interrupt.h file)
    //from system header file: #define INT_UART1 22
    IntRegister(INT_UART1, UART1_Handler);

    //globally allow CPU to service interrupts (see interrupt.h file)
    IntMasterEnable();

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
    }
    else {
        rdata = (char) (UART1_DR_R & 0xFF);
    }
    return rdata;
}

command_t uart_receive_nonblocking(char *data) {
    (*data) = command_byte;
    if (command_flag != 0) {
        return UART_Command;
    }
    return NO_COMMAND;
}

void uart_sendStr(uint8_t *data){
    int i = 0;
    int length = sizeof(data) / sizeof(uint8_t);
    for (i = 0; i < length; i++) {
        uart_sendChar(data[i]);
    }
}

// Interrupt handler for receive interrupts
void UART1_Handler(void) {
    //check if handler called due to RX event
    if (UART1_RIS_R & BIT4) {
        //byte was received in the UART data register
        //clear the RX trigger flag (clear by writing 1 to ICR)
        char dump;
        int i = 0;
        for (i = 0; i < 8; i++) {
            UART_Byte_Stream[i] = UART1_DR_R & 0xFF;
        }
        for (i = 0; i < 8; i++) {
            dump = UART1_DR_R & 0xFF;
        }
        UART_Payload_Recieved = true;
    }
    UART1_ICR_R |= BIT4;
}
