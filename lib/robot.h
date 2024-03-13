/*
 * robot.h
 *
 *  Created on: Apr 23, 2023
 *      Author: Randall Larson
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include <stdint.h>
#include <stdbool.h>
#include <inc/tm4c123gh6pm.h>

#include "sensor.h"
#include "open_interface.h"

#define ROBOT_OP_STATUS         1
#define ROBOT_OP_FORWARD        2
#define ROBOT_OP_BACKWARD       3
#define ROBOT_OP_LEFT           4
#define ROBOT_OP_RIGHT          5
#define ROBOT_OP_SCAN           6
#define ROBOT_OP_SCAN_STATUS    7
#define ROBOT_OP_SCAN_COMPLETE  8
#define ROBOT_OP_DESTINATION    9


typedef enum {
    IDLE,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    COLLISION,
    DESTINATION,
    CLIFF,
    EDGE
} robot_state_t;

extern volatile robot_state_t Robot_State;


// Initialize the UART communication between the Tiva MCU and the iRobot
void robot_init(void);

void robot_close(void);

double robot_forward_distance(unsigned int, unsigned int);
double robot_backward_distance(unsigned int, unsigned int);
double robot_left_distance(unsigned int, unsigned int);
double robot_right_distance(unsigned int, unsigned int);

void robot_status(void);
void robot_scan_status(sensor_reading_t*);
void robot_scan_complete(void);
void robot_destination_reached(void);

#endif /* ROBOT_H_ */
