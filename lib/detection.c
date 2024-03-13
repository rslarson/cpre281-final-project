/*
 * detection.c
 *
 *  Created on: Feb 12, 2023
 *      Authors:
 *          Mohammed Abdalgader (mohd19@iastate.edu)
 *          Randall Larson (rslarson@iastate.edu)
 */

#include <stdlib.h>
#include <string.h>
#include <Math.h>

#include "detection.h"
#include "cyBot_Scan.h"
#include "uart.h"
#include "lcd.h"

typedef struct{
    int angle;
    float ping_distance;
    float ir_adc;
} data_point_t;

object_t newObject(int angle, double distance, double width) {
    object_t out = {.angle = angle, .distance = distance, .width = width};
    return out;
}

data_point_t scanPoint(int angle, cyBOT_Scan_t *scan) {
    float ping_distance = 0;
    float ir_adc = 0;
    int i = 0;

    for (i = 0; i < sampling; i++) {
        cyBOT_Scan(angle, scan);
        ping_distance += scan->sound_dist;
        ir_adc += scan->IR_raw_val;
    }
    data_point_t out = {.angle=angle, .ping_distance=ping_distance, .ir_adc=ir_adc};

    return out;

}

void scanObjects(object_t *objectsArr, cyBOT_Scan_t *scan) {
    const int data_points = (scan_field / angle_stepping) + 1;

    int i = 0;
    int j = 0;
    int data_index = 0;
    int object_index = 0;
    int left_index = -1;
    int right_index = -1;
    int object_angle = 0;

    char *angle_msg = calloc(20, sizeof(char));
    char *scan_results = calloc(50, sizeof(char));
    data_point_t *data = calloc(data_points, sizeof(data_point_t));
    char scan_header[] = "Degrees\tPING Distance (cm)\tIR Value\n\r"; //UART debug header

    float avg_distance = 0;
    double data_point_slope = 0;
    double object_angle_rad = 0;
    double object_width = 0;

    //Print Scanning Message on LCD
    lcd_clear();
    lcd_printf("Detecting objects...");

    //Print debug header to UART
    uart_sendStr(scan_header);

    cyBOT_Scan(0, NULL);
    for (i = 0; i <= scan_field; i += angle_stepping) {
        sprintf(angle_msg, "Scanning @ %d deg.", i);
        lcd_setCursorPos(0,1);
        lcd_puts(angle_msg);
        data[data_index] = scanPoint(i, scan);
        sprintf(
                scan_results,
                "%-7d\t%-18f\t%-8.3f\n\r",
                data[data_index].angle,
                data[data_index].ping_distance,
                data[data_index].ir_adc
                );
        uart_sendStr(scan_results);
        data_index++;
    }

    lcd_home();
    lcd_printf("Measuring Distance");

    for (i = 1; i < data_points; i++) {
        data_point_slope = (data[i].ir_adc - data[i-1].ir_adc) / (data[i].angle - data[i-1].angle);
        if (data_point_slope >= 100 && left_index == -1) {
            left_index = i-1;
        }
        else if (data_point_slope <= -100 && left_index != -1 && right_index == -1) {
            right_index = i-1;
        }
        if (left_index != -1 && right_index != -1) {
            object_angle = data[left_index].angle;
            for (j = left_index + 1; j < right_index; j++) {
                if (data[j].ir_adc < data[j-1].ir_adc) {
                    object_angle = data[j].angle;
                }
            }
            lcd_setCursorPos(0,1);
            sprintf(angle_msg, "Object @ %d deg", object_angle);
            lcd_puts(angle_msg);
            for (j = 0; j < sampling; j++) {
                cyBOT_Scan(object_angle, scan);
                avg_distance += scan->sound_dist;
            }
            avg_distance = avg_distance / sampling;
            if (avg_distance <= 100) {
                object_angle_rad = ((data[right_index].angle - data[left_index].angle) * M_PI) / 180;
                object_width = sqrt(pow(data[left_index].ping_distance,2) + pow(data[right_index].ping_distance,2)
                                    - (2 * data[left_index].ping_distance * data[right_index].ping_distance * cos(object_angle_rad)));
                if(object_width >= minimum_width_cm && object_index < 20) {
                    objectsArr[object_index] = newObject(object_angle, avg_distance, object_width);
                    object_index++;
                }
            }
            left_index = -1;
            right_index = -1;
        }
    }
}
