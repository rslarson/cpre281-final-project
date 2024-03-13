/*
 * robot.c
 *
 *  Created on: Apr 23, 2023
 *      Author: Randall Larson
 *
 */

#include <Math.h>
#include <stdbool.h>

#include "robot.h"
#include "uart-interrupt.h"


volatile robot_state_t Robot_State = IDLE;


static bool robot_initialized = false;
static oi_t *sensor_data;


void robot_init() {
    if (robot_initialized) {
        return;
    }

    sensor_data = oi_alloc();
    oi_init(sensor_data);
    robot_initialized = true;
}

void robot_close() {
    oi_free(sensor_data);
    Robot_State = IDLE;
    robot_initialized = false;
}

double robot_forward_distance(unsigned int distance_mm, unsigned int velocity) {
    // Clear OI Sensor Registers
    oi_update(sensor_data);
    oi_update(sensor_data);


    // Checks that the robot is currently not in contact with anything
    if (sensor_data->bumpLeft || sensor_data->bumpRight) {
        return 0;
    }

    // Checks that the robot is not near a cliff
    if (sensor_data->cliffFrontLeft || sensor_data->cliffFrontRight || sensor_data->cliffLeft || sensor_data->cliffRight)  {
        return 0;
    }

    double distance_traveled = 0;
    Robot_State = FORWARD;
    oi_setWheels(velocity, velocity);
    while (Robot_State == FORWARD) {
        oi_update(sensor_data);
        distance_traveled += sensor_data->distance;
        if (distance_traveled >= distance_mm) {
            oi_setWheels(0,0);
            Robot_State = IDLE;
            robot_status();
        }
        else if ((sensor_data->bumpLeft || sensor_data->bumpRight)) {
            oi_setWheels(0,0);
            Robot_State = COLLISION;
            robot_status();

            // In the instance that state becomes COLLISION, autonomous response to backup.
            robot_backward_distance(50, 50);
            return distance_traveled;
        }
        else if ((sensor_data->cliffFrontLeftSignal < 900) || (sensor_data->cliffFrontRightSignal < 900) ||
                sensor_data->cliffLeftSignal < 900 || sensor_data->cliffRightSignal < 900) {
                oi_setWheels(0,0);
                Robot_State = CLIFF;
                robot_status();

                // In the instance that state becomes COLLISION, autonomous response to backup.
                robot_backward_distance(50, 50);
                return distance_traveled;
        }
        else if ((sensor_data->cliffFrontLeftSignal > 2600) || (sensor_data->cliffFrontRightSignal > 2600)) {
            oi_setWheels(0,0);
            Robot_State = EDGE;
            robot_status();

            // In the instance that state becomes COLLISION, autonomous response to backup.
            robot_backward_distance(50, 50);
            return distance_traveled;
        }
        else if ((sensor_data->cliffFrontLeftSignal < 1400) || (sensor_data->cliffFrontRightSignal < 1400)) {
            if(Robot_State != DESTINATION) {
                oi_setWheels(0,0);
                Robot_State = DESTINATION;
            }

            robot_status();
            robot_destination_reached();
            timer_waitMillis(1000);

            if(Robot_State == DESTINATION) {
                double destination_distance = 0;
                oi_update(sensor_data);
                oi_update(sensor_data);
                oi_setWheels(100, 100);
                while (destination_distance < 300){
                    oi_update(sensor_data);
                    destination_distance += sensor_data->distance;
                    timer_waitMillis(100);
                }
                oi_setWheels(0, 0);
                Robot_State = IDLE;
            }
            return distance_traveled;
        }
    }
    return distance_traveled;
}

double robot_backward_distance(unsigned int distance_mm, unsigned int velocity) {
    double distance_traveled = 0;

    Robot_State = BACKWARD;

    // Clear OI Sensor Registers
    oi_update(sensor_data);
    oi_update(sensor_data);


    oi_setWheels(-velocity, -velocity);
    while (Robot_State == BACKWARD) {
        oi_update(sensor_data);
        distance_traveled += sensor_data->distance;
        if (fabs(distance_traveled) >= distance_mm) {
            oi_setWheels(0,0);
            Robot_State = IDLE;
        }
        else if (sensor_data->wheelDropLeft || sensor_data->wheelDropRight) {
            oi_setWheels(0,0);
            Robot_State = COLLISION;
        }
    }
    return distance_traveled;
}

double robot_left_distance(unsigned int deg, unsigned int velocity) {
    double angle_turned = 0;

    Robot_State = LEFT;

    // Clear OI Sensor Registers
    oi_update(sensor_data);
    oi_update(sensor_data);


    oi_setWheels(velocity, -velocity);
    while (Robot_State == LEFT) {
        oi_update(sensor_data);
        angle_turned += sensor_data->angle;
        if (angle_turned >= deg) {
            oi_setWheels(0,0);
            Robot_State = IDLE;
        }
    }
    robot_status();
    return angle_turned;
}


double robot_right_distance(unsigned int deg, unsigned int velocity) {
    double angle_turned = 0;

   Robot_State = RIGHT;

   // Clear OI Sensor Registers
   oi_update(sensor_data);
   oi_update(sensor_data);


   oi_setWheels(-velocity, velocity);
   while (Robot_State == RIGHT) {
       oi_update(sensor_data);
       angle_turned -= sensor_data->angle;
       if (fabs(angle_turned) >= deg) {
           oi_setWheels(0,0);
           Robot_State = IDLE;
       }
   }
   robot_status();
   return fabs(angle_turned);
}

void robot_status() {
    oi_update(sensor_data);
    outputByteStream[0] = ROBOT_OP_STATUS;

    outputByteStream[1] = 0x9;


    outputByteStream[2] |= sensor_data->wheelDropRight;
    outputByteStream[2] |= sensor_data->wheelDropLeft << 1;
    outputByteStream[2] |= sensor_data->bumpRight << 2;
    outputByteStream[2] |= sensor_data->bumpLeft << 3;
    if (Robot_State == CLIFF){
        outputByteStream[2] |= 0x7 << 4;
    }
    else if (Robot_State == EDGE) {
        outputByteStream[2] |= 0x8 << 4;
    }

    outputByteStream[3] = sensor_data->cliffLeftSignal >> 8;
    outputByteStream[4] = sensor_data->cliffLeftSignal & 0xFF;

    outputByteStream[5] = sensor_data->cliffFrontLeftSignal >> 8;
    outputByteStream[6] = sensor_data->cliffFrontLeftSignal & 0xFF;

    outputByteStream[7] = sensor_data->cliffFrontRightSignal >> 8;
    outputByteStream[8] = sensor_data->cliffFrontRightSignal & 0xFF;

    outputByteStream[9] = sensor_data->cliffRightSignal >> 8;
    outputByteStream[10] = sensor_data->cliffRightSignal & 0xFF;

    int i = 0;
    for (i = 0; i < 11; i++) {
        uart_sendChar(outputByteStream[i]);
        outputByteStream[i] = 0;
    }
}

void robot_scan_status(sensor_reading_t *data) {

    outputByteStream[0] = ROBOT_OP_SCAN_STATUS;

    outputByteStream[1] = 0x4;

    outputByteStream[2] = data->angle;

    outputByteStream[3] = data->ir_adc >> 8;
    outputByteStream[4] = data->ir_adc & 0xFF;

    outputByteStream[5] = data->sonar_distance >> 8;
    outputByteStream[6] = data->sonar_distance & 0xFF;

    int i = 0;
    for (i = 0; i < 11; i++) {
        uart_sendChar(outputByteStream[i]);
        outputByteStream[i] = 0;
    }
}

void robot_scan_complete() {

    outputByteStream[0] = ROBOT_OP_SCAN_COMPLETE;

    int i = 0;
    for (i = 0; i < 11; i++) {
        uart_sendChar(outputByteStream[i]);
        outputByteStream[i] = 0;
    }
}

void robot_destination_reached() {

    outputByteStream[0] = ROBOT_OP_DESTINATION;

    int i = 0;
    for (i = 0; i < 11; i++) {
        uart_sendChar(outputByteStream[i]);
        outputByteStream[i] = 0;
    }
}
