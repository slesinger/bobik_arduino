#ifndef robot_config_types_h
#define robot_config_types_h

#include <stdint.h>

struct RotationSensor_t {
    uint8_t spi_cs_pin;
    uint16_t zero_position;
};

struct RotationMotor_t {
    uint8_t in1;
    uint8_t in2;
    uint8_t ena;
};

struct DriveMotor_t {
    uint8_t in1;
    uint8_t in2;
    uint8_t ena;
};

struct Caster_t {
    RotationSensor_t rotation_sensor;
    RotationMotor_t rotation_motor;
    DriveMotor_t drive_motor;
};

struct Base_t {
    Caster_t caster_fl;
    Caster_t caster_fr;
    Caster_t caster_r;
};



#endif
