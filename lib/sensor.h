/*
 * sensor.h
 *
 *  Created on: Mar 29, 2023
 *      Author: rslarson
 */

#include <stdint.h>
#include <stdbool.h>

#include "driverlib/interrupt.h"

#ifndef SENSOR_H_
#define SENSOR_H_

typedef struct {
    uint8_t angle;
    uint16_t ir_adc;
    uint16_t sonar_distance;
} sensor_reading_t;

extern int LEFT_CALIBRATION_VALUE;
extern int RIGHT_CALIBRATION_VALUE;

void sensor_init(uint8_t mask);
void sensor_scan(sensor_reading_t*, int, int);

void adc_init(void);
int adc_read(void);

void sonar_init(void);
unsigned short sonar_pulse(void);
void TIMER3B_Handler(void);

void servo_init(void);
uint32_t servo_move(int);
void servo_lcd_control(void);
void servo_calibration(void);

#endif /* SENSOR_H_ */
