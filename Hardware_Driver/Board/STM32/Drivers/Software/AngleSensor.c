#include "AngleSensor.h"

/**
 * @brief 角度归一化 (0~360)
 * @param ang 角度
*/
float normalizeAng(float ang) {
    while(ang > 360) {
        ang -= 360;
    }
    while(ang < 0) {
        ang += 360;
    }
    return ang;
}

void AngleSensor_Init(AngleSensor_t *sensor, uint8_t id, float offset) {
    sensor->id = id;
    sensor->offset = offset;
    sensor->raw_val = 0;
    sensor->angle = 0;
    sensor->angle_rad = 0;
    return;
}

void updateAngle(AngleSensor_t *sensor, uint32_t raw_val) {
    sensor->raw_val = raw_val;
    sensor->angle = (float)raw_val * 360.0f / 4095.0f - sensor->offset;
    sensor->angle = normalizeAng(sensor->angle);
    sensor->angle_rad = sensor->angle * PI / 180.0f;
}

