/*
 * sensor.c
 *
 *  Created on: Mar 29, 2023
 *      Author: rslarson
 */



#include <inc/tm4c123gh6pm.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "button.h"
#include "lcd.h"
#include "sensor.h"
#include "Timer.h"
#include "robot.h"
#include "uart-interrupt.h"

#define BIT0 0x01
#define BIT1 0x02
#define BIT2 0x04
#define BIT3 0x08
#define BIT4 0x10
#define BIT5 0x20
#define BIT6 0x40
#define BIT7 0x80
#define BIT8 0x100
#define BIT9 0x200
#define BIT10 0x400
#define BIT11 0x800

static bool adc_initialized = false;
static bool sonar_initialized = false;
static bool servo_initialized = false;
static bool sensor_initialized = false;

// 80MHz == 62.5ns period
const float clock_period = 62.5E-9;
const unsigned long clock_speed = 16E6;
const unsigned long pwm_period = 360000;

int LEFT_CALIBRATION_VALUE = 32000;
int RIGHT_CALIBRATION_VALUE = 16000;

double pwm_slope = 88.889;

// Speed of sound is 343 m/s
const int sound_velocity = 343;

// Sonar Timer Variables
volatile unsigned long timerStart = 0;
volatile unsigned long timerEnd = 0;
volatile long timerDelta = 0;

// Tracks positive or negative edge or Sonar RX
volatile enum{LOW, HIGH, DONE} STATE = LOW;

void sensor_init(uint8_t mask) {
    if (sensor_initialized) {
        return;
    }

    if (mask & 0x01) {
        servo_init();
    }

    if (mask & 0x02) {
        sonar_init();
    }

    if (mask & 0x04) {
        adc_init();
    }

    if (adc_initialized || sonar_initialized || servo_initialized){
        sensor_initialized = true;
    }
}

void sensor_scan(sensor_reading_t *data, int start_deg, int end_deg){
    int i = 0;
    for (i = start_deg; i <= end_deg; i++) {
        if (servo_initialized) {
            servo_move(i);
            timer_waitMillis(75);
            data->angle = i;
        }

        if (adc_initialized) {
            data->ir_adc = adc_read();
        }

        if (sonar_initialized) {
            unsigned short pulse = 0;
            int j = 0;
            for (j = 0; j < 3; j++) {
                pulse += sonar_pulse();
            }
            pulse = pulse / 3;
            data->sonar_distance = pulse;
        }
        robot_scan_status(data);
    }
    robot_scan_complete();
}

void adc_init(void)
{
    if (adc_initialized) {
        return;
    }

    SYSCTL_RCGCADC_R |= 0x1;
    SYSCTL_RCGCGPIO_R |= 0x2;
    GPIO_PORTB_DEN_R   &= 0b11101111;
    GPIO_PORTB_AFSEL_R |= 0b00010000;
    GPIO_PORTB_AMSEL_R |= 0b00010000;

    // Using SS3
    ADC0_ACTSS_R  &= 0b0111;    //disabling SS3
    ADC0_EMUX_R  &= 0x0FFF;    //
//    ADC0_EMUX_R |= 0x4000;  // Setting trigger to just GPIO
    ADC0_SSMUX3_R = 0xA;
    ADC0_SSCTL3_R = 0b0110;      // Interrupt and end bit
    ADC0_IM_R    |= 0b1000;      // Mask for SS3
    ADC0_ACTSS_R |= 0b1000;    // Enable SS3
//    NVIC_EN0_R |= 0x00020000; //17

    adc_initialized = true;
}

int adc_read(void)
{
    ADC0_PSSI_R |= 0b1000;
    while(~ADC0_RIS_R & 0b1000)
    {}
    ADC0_ISC_R |= 0b1000;
    return ADC0_SSFIFO3_R & 0xFFF;
}


void sonar_tx(void) {
    // Enable PB3 as GPIO output
    GPIO_PORTB_DIR_R |= BIT3;

    // Disable Alternate Function on PB3
    GPIO_PORTB_AFSEL_R &= ~(1 << 3);

    // Set PB3 to Pull-Down
    GPIO_PORTB_PDR_R |= BIT3;

    // Enable Digital functionality on PB3
    GPIO_PORTB_DEN_R |= BIT3;

    // Force PB3 to LOW
    GPIO_PORTB_DATA_R &= ~(1 << 3);

    // Set PB3 to HIGH
    GPIO_PORTB_DATA_R |= BIT3;

    //Wait 5 microseconds
    timer_waitMicros(5);

    // Set PB3 to LOW
    GPIO_PORTB_DATA_R &= ~(1 << 3);
}

void sonar_rx(void) {
    STATE = LOW;
    timerDelta = 0;

    // Set PB3 to Alternate Function
    GPIO_PORTB_AFSEL_R |= BIT3;

    // Select PB3 - Timer 3B Alternate Function
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB3_T3CCP1;

    // Disable timer while configuring
    TIMER3_CTL_R &= ~(TIMER_CTL_TBEN);

    // Configure timer Prescaller
    TIMER3_TBPR_R = TIMER_TBPR_TBPSR_M;

    // Configure timer starting value
    TIMER3_TBILR_R = 0x0000FFFF;

    // Enable the timer
    TIMER3_CTL_R |= TIMER_CTL_TBEN;
}

void sonar_init(void) {
    if (sonar_initialized) {
        return;
    }

    // enable clock on Timer3B
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R3;

    // enable clock to GPIO port B
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;

    // Disable timer while configuring
    TIMER3_CTL_R &= ~(TIMER_CTL_TBEN);

    // Configure Timer into 16 bit mode
    TIMER3_CFG_R |= TIMER_CFG_16_BIT;

    // Configure Timer for Capture (edge-detect) mode
    TIMER3_TBMR_R |= (TIMER_TBMR_TBMR_CAP | TIMER_TBMR_TBCMR);

    // Configure Timer to Count Down
    TIMER3_TBMR_R &= ~(TIMER_TBMR_TBCDIR);

    // Configure Timer events for positive and negative edge
    TIMER3_CTL_R |= TIMER_CTL_TBEVENT_BOTH;

    // Configure timer interrupt
    TIMER3_IMR_R |= TIMER_IMR_CBEIM;

    // NVIC setup: set priority of TIMER3B
    // Interrupt 36
    NVIC_PRI9_R = (NVIC_PRI9_R & 0xFFFFFF0F) | 0x00000020;

    //NVIC setup: enable interrupt for TIMER3B
    NVIC_EN1_R |= BIT4;

    //Register Interrupt handler
    IntRegister(INT_TIMER3B, TIMER3B_Handler);
    IntMasterEnable();

    sonar_initialized = true;
}

void TIMER3B_Handler(void) {
    if(TIMER3_MIS_R & TIMER_MIS_CBEMIS) {
        if (STATE == LOW) {
            timerStart = (TIMER3_TBPS_R << 16) | (TIMER3_TBR_R & 0xffff);
            STATE = HIGH;
        }
        else if (STATE == HIGH) {
            TIMER3_CTL_R &= ~(TIMER_CTL_TBEN);
            timerEnd = (TIMER3_TBPS_R << 16) | (TIMER3_TBR_R & 0xffff);
            timerDelta = timerStart - timerEnd;
            STATE = DONE;
        }
    }
    TIMER3_ICR_R |= TIMER_ICR_CBECINT;
}

unsigned short sonar_pulse(void) {
    sonar_tx();
    sonar_rx();

    while (STATE != DONE) {
        if (STATE == DONE) {
            break;
        }
    }

    float distance = (timerDelta / 2) * clock_period * sound_velocity;

    unsigned short out = distance * 100;
    if (out > 250) {
        out = 250;
    }

    // Wait 750 Microseconds - minimum time between pulses per datasheet
    timer_waitMicros(750);
    return out;
}

void servo_init(void) {
    if (servo_initialized) {
        return;
    }

    pwm_slope = (double)(LEFT_CALIBRATION_VALUE - RIGHT_CALIBRATION_VALUE) / 180;

    // Enable Clock on Timer1B
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;

    // Enable clock to GPIO port B
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;

    // Enable Digital functionality on PB5
    GPIO_PORTB_DEN_R |= BIT5;

    // Set PB5 to Digital output.
    GPIO_PORTB_DIR_R |= BIT5;

    // Set PB5 to Alternate Function
    GPIO_PORTB_AFSEL_R |= BIT5;

    // Select PB3 - Timer 3B Alternate Function
    GPIO_PORTB_PCTL_R |= GPIO_PCTL_PB5_T1CCP1;

    // Disable timer while configuring
    TIMER1_CTL_R |= ~(TIMER_CTL_TBEN);

    // Configure Timer into 16 bit mode
    TIMER1_CFG_R |= TIMER_CFG_16_BIT;

    // Configure Timer1B for PWM Count-Down mode
    TIMER1_TBMR_R &= ~(TIMER_TBMR_TBCMR);
    TIMER1_TBMR_R |= (TIMER_TBMR_TBAMS | TIMER_TBMR_TBMR_PERIOD);

    // Configure output state of PWM signal
    TIMER1_CTL_R &= ~(TIMER_CTL_TBPWML);

    // Configure Timer 1 initial value (pwm_pulse_clock_cycles == 0x4E200)
    TIMER1_TBILR_R = pwm_period & 0xFFFF;
    TIMER1_TBPR_R = pwm_period >> 16;

    // Move to home position
    servo_move(0);

    servo_initialized = true;
}

uint32_t servo_move(int deg) {
    uint32_t cycles = 0;
    cycles = (pwm_slope * deg) + RIGHT_CALIBRATION_VALUE;

    uint32_t match_cycles = pwm_period - cycles;

    TIMER1_CTL_R &= ~(TIMER_CTL_TBEN);
    TIMER1_TBPMR_R = match_cycles >> 16;
    TIMER1_TBMATCHR_R = match_cycles & 0xFFFF;
    TIMER1_CTL_R |= TIMER_CTL_TBEN;
    return cycles;
}

void servo_lcd_control(void) {
    char *dir_msg = calloc(20, sizeof(char));
    char *target_msg = calloc(20, sizeof(char));
    char *deg_msg = calloc(20, sizeof(char));
    uint8_t servo_pos = 0;
    bool cw_dir = true;

    // Previous state of the button3 and button4; true if pushed, false if released.
    bool button3_ps = false;
    bool button4_ps = false;

    bool lcd_update = true;
    bool move_servo = true;
    uint8_t buttons_pushed = 0;

    sprintf(dir_msg, "Dir: %15s", "Clockwise");

    //Return servo to home (0 deg)
    servo_move(servo_pos);

    while(UART_Command != PROGRAM_STOP) {
        // Get pushed buttons
        buttons_pushed = button_getButton();

        //Button3 - Servo Direction
        if (buttons_pushed & 0x04 && !button3_ps) {
            button3_ps = true;
            if (cw_dir == true) {
                cw_dir = false;
                sprintf(dir_msg, "Dir: %15s", "Clockwise");
            }
            else {
                cw_dir = true;
                sprintf(dir_msg, "Dir: %15s", "Cntr-Clockwise");
            }
            lcd_update = true;
        }
        else if (!(buttons_pushed & 0x04) && button3_ps) {
            button3_ps = false;
        }

        //Button4 - Move to Servo Limit
        if (buttons_pushed & 0x08 && !button4_ps) {
            button4_ps = true;
            if (cw_dir == true){
                servo_pos = 5;
            }
            else{
                servo_pos = 175;
            }
            lcd_update = true;
            move_servo = true;
        }
        else if (!(buttons_pushed & 0x08) && button4_ps) {
            button4_ps = false;
        }

        // Button1 - Move 1 deg, Button2 - Move 5 deg, Both or none - do nothing.
        if ((buttons_pushed & 0x01) && !(buttons_pushed & 0x02)) {
            if (cw_dir == true && servo_pos > 0){
                servo_pos -= 1;
            }
            else if(cw_dir == false && servo_pos < 180){
                servo_pos += 1;
            }
            lcd_update = true;
            move_servo = true;

        }
        else if (!(buttons_pushed & 0x01) && (buttons_pushed & 0x02)) {
            if (cw_dir == true) {
                if (servo_pos >= 5){
                    servo_pos -= 5;
                }
                else if(servo_pos < 5 && servo_pos > 0) {
                    servo_pos -= servo_pos;
                }
            }
            else {
                if (servo_pos <= 175){
                    servo_pos += 5;
                }
                else if(servo_pos > 175 && servo_pos < 180){
                    servo_pos += (180 - servo_pos);
                }
            }
            lcd_update = true;
            move_servo = true;
        }

        //Move Servo
        if (move_servo) {
            sprintf(target_msg, "Target: %6d", servo_move(servo_pos));
            sprintf(deg_msg, "Target deg: %3d", servo_pos);
            lcd_update = true;
        }

        // Update LCD
        if (lcd_update) {
            //lcd_clear();
            lcd_home();
            lcd_puts(dir_msg);
            lcd_setCursorPos(0,2);
            lcd_puts(deg_msg);
            lcd_setCursorPos(0,3);
            lcd_puts(target_msg);
            timer_waitMillis(150);
        }
        lcd_update = false;
        move_servo = false;
    }
}

void servo_calibration(void) {
    if (!sensor_initialized) {
        lcd_printf("Servo not\ninitialized!");
        return;
    }
    int state = 0;
    int servo_pos = 0;
    uint8_t buttons_pushed = 0;
    char *tick_msg = malloc(20 * sizeof(char));
    uint32_t tick_val = 0;
    uint32_t left_val = 0;
    uint32_t right_val = 0;

    lcd_clear();
    lcd_home();
    lcd_puts("B1: Left   B2: Right");
    lcd_setCursorPos(0,1);
    lcd_puts("B4 - Right lim Value");

    while (state < 2) {
        buttons_pushed = button_getButton();
        if (buttons_pushed & 0x01) {
            servo_pos++;
            tick_val = servo_move(servo_pos);
        }
        else if (buttons_pushed & 0x02) {
            servo_pos--;
            tick_val = servo_move(servo_pos);
        }
        else if (buttons_pushed & 0x08 && state == 0) {
            state++;
            right_val = tick_val;
            sprintf(tick_msg,"Right Val: %d", right_val);
            lcd_setCursorPos(0,3);
            lcd_puts(tick_msg);
            lcd_setCursorPos(0,1);
            lcd_puts("B4 - Left lim Value");
            timer_waitMillis(100);
        }
        else if (buttons_pushed & 0x08 && state == 1) {
            state++;
            left_val = tick_val;
            sprintf(tick_msg,"Left Val: %d", left_val);
            lcd_setCursorPos(0,3);
            timer_waitMillis(100);
        }
        timer_waitMillis(25);
    }

    lcd_clear();
    lcd_home();
    lcd_puts(tick_msg);
    lcd_setCursorPos(0,1);
    sprintf(tick_msg,"Right Val: %d", right_val);
    lcd_puts(tick_msg);
}
