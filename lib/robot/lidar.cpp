#include "lidar.h"
#include <Arduino.h>

Lidar::Lidar(Lidar_t lidar_cfg)
{
    pinMode(lidar_cfg.motor_pwm_pin, OUTPUT);
    analogWrite(LIDAR_PWM, LIDAR_PWM);
}

Lidar::~Lidar()
{
}
