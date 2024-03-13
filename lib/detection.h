/*
 * detection.h
 *
 *  Created on: Feb 12, 2023
 *      Authors:
 *          Mohammed Abdalgader (mohd19@iastate.edu)
 *          Randall Larson (rslarson@iastate.edu)
 */

#ifndef DETECTION_H_
#define DETECTION_H_

#include "cyBot_Scan.h"

typedef struct{
    int angle;
    double distance;
    double width;
} object_t;

int maximum_detection_cm;
int minimum_width_cm;
int scan_field;
int angle_stepping;
int sampling;

object_t newObject(int angle, double distance, double width);
void scanObjects(object_t *objectsArr, cyBOT_Scan_t *scan);

#endif /* DETECTION_H_ */
