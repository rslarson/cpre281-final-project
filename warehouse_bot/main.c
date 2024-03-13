#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>

#include <inc/tm4c123gh6pm.h>
#include "uart-interrupt.h"
//#include "button.h"
//#include "lcd.h"
#include "Timer.h"
#include "robot.h"
#include "open_interface.h"
#include "sensor.h"

uint16_t parseInt(char *theInt) {
    return (theInt[0] << 8) | theInt[1];
}

uint8_t parseInt8(char *theInt) {
    return (theInt[0] << 8) | theInt[1];
}


int main(void)
{

    timer_init(); // Initialize Timer, needed before any LCD screen functions can be called
                  // and enables time functions (e.g. timer_waitMillis)

    uart_init();  //initialize uart interrupt

    robot_init();   //initialize robot state at STOPPED or IDLE (depends on current state)

    //lcd_init();

    //button_init();

    LEFT_CALIBRATION_VALUE = 36177;
    RIGHT_CALIBRATION_VALUE = 7377;
    sensor_init(7);
    //servo_calibration();

    sensor_reading_t *sensor_data = calloc(1, sizeof(sensor_reading_t));  //char read in from PuTTy

    bool run = true;



    robot_state_t prv_state = Robot_State;

    uint16_t distance = 0;
    uint16_t velocity = 0;
    uint8_t degree = 0;
    uint8_t startAngle = 0;
    uint8_t endAngle = 0;

//TEST CLIFF SENSORS:
//    data = uart_receive();
//
//    if(data = 'w') {
//        robot_forward_distance(100,100);
//    }

    while(run) {

        if(UART_Payload_Recieved) {
            switch(*UART_Byte_Stream) {
                        case ROBOT_OP_FORWARD :
                            distance = parseInt(UART_Byte_Stream + 2);
                            velocity = parseInt(UART_Byte_Stream + 4);

                            robot_forward_distance(distance, velocity);
                            UART_Payload_Recieved = false;
                            break;

                        case ROBOT_OP_BACKWARD :
                            distance = parseInt(UART_Byte_Stream + 2);
                            velocity = parseInt(UART_Byte_Stream + 4);

                            robot_backward_distance(distance,velocity);
                            UART_Payload_Recieved = false;
                            break;

                        case ROBOT_OP_LEFT :
                            degree = UART_Byte_Stream[2];
                            velocity = parseInt(UART_Byte_Stream + 3);

                            robot_left_distance(degree,velocity);
                            UART_Payload_Recieved = false;
                            break;

                        case ROBOT_OP_RIGHT :
                            degree = UART_Byte_Stream[2];
                            velocity = parseInt(UART_Byte_Stream + 3);

                            robot_right_distance(degree,velocity);
                            UART_Payload_Recieved = false;
                            break;

                        case ROBOT_OP_SCAN :
                            startAngle = UART_Byte_Stream[2];
                            endAngle = UART_Byte_Stream[3];

                            sensor_scan(sensor_data, startAngle, endAngle);
                            UART_Payload_Recieved = false;
                            break;
                        case ROBOT_OP_STATUS :
                            robot_status();
                            UART_Payload_Recieved = false;
                            break;
                        }
        }
    }
    robot_close();

    return 0;
}
