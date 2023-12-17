#ifndef __ANGLE_SENSOR_H
#define __ANGLE_SENSOR_H

#include "sys.h"

typedef struct __AngleSensor_t {
    uint8_t id;
    uint32_t raw_val;           // 0~4095
    float angle;                // 0~360
    float angle_rad;            // 0~2*PI
    float offset;
} AngleSensor_t;

void AngleSensor_Init(AngleSensor_t *sensor, uint8_t id, float offset);
void updateAngle(AngleSensor_t *sensor, uint32_t raw_val);

#endif

