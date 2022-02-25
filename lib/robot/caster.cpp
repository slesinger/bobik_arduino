#include "caster.h"
#include "utils.h"
#include <unity.h> 

#define AVG_SIZE 7 //how many sensor readings to average
#define ROTATION_TOLERANCE 30
#define PWM_MAX 255

Caster::Caster(Caster_t caster_cfg)
{
    cfg = caster_cfg;
    rotation_sensor = new AS5048A(cfg.rotation_sensor.spi_cs_pin, cfg.rotation_sensor.zero_position); // SPI cable select pin, zero angle value
    rotation_sensor->init();

    // read first values which are inaccurate
    for (int i=0; i<20; i++) {
        getRotation();
    }

    // Rotation motor pins
    pinMode(cfg.rotation_motor.in1, OUTPUT);
    pinMode(cfg.rotation_motor.in2, OUTPUT);
    pinMode(cfg.rotation_motor.ena, OUTPUT);

    rotation_target = 0;
    pid_prev_rotation = 0;
    pid_i = 0;

}

int16_t Caster::getRotation()
{
    long sum = 0;
    for (int i=0; i<AVG_SIZE; i++) {
        uint16_t raw = rotation_sensor->getRawRotation();
        int16_t rotation = raw - cfg.rotation_sensor.zero_position;
        if(rotation > 8191) rotation = -((0x3FFF)-rotation); //more than -180
        sum += rotation;
    }
    return (int16_t)(sum / (float)AVG_SIZE);
}

void Caster::setRotationTarget(int16_t rotation_units)
{
    // TODO impose limits
    rotation_target = rotation_units;
}

void Caster::pingRotationMotor()
{
    #define TIME_TO_ROTATE 200
    int pwm = PWM_MAX;
    digitalWrite(cfg.rotation_motor.in1, sign1(pwm));
    digitalWrite(cfg.rotation_motor.in2, sign2(pwm));
    analogWrite(cfg.rotation_motor.ena, abs(pwm));
    delay(TIME_TO_ROTATE);
    pwm = 0;
    digitalWrite(cfg.rotation_motor.in1, sign1(pwm));
    digitalWrite(cfg.rotation_motor.in2, sign2(pwm));
    analogWrite(cfg.rotation_motor.ena, abs(pwm));
    return;
    delay(1000);
    pwm = -PWM_MAX;
    digitalWrite(cfg.rotation_motor.in1, sign1(pwm));
    digitalWrite(cfg.rotation_motor.in2, sign2(pwm));
    analogWrite(cfg.rotation_motor.ena, abs(pwm));
    delay(TIME_TO_ROTATE);
    pwm = 0;
    digitalWrite(cfg.rotation_motor.in1, sign1(pwm));
    digitalWrite(cfg.rotation_motor.in2, sign2(pwm));
    analogWrite(cfg.rotation_motor.ena, abs(pwm));
    delay(1000);
}

void Caster::execute()
{
    char buffer [128];
    // PID controller
    int16_t current = this->getRotation();
    int16_t p = rotation_target - current;
    long effort = p + (pid_i / 2);
    // Full speed rotation changes 145 rotation unit per 50ms
    int pwm = map_cut(abs(effort), 
        ROTATION_TOLERANCE, // do not move if close enough to target
        600, // if higher than that full thrust
        100, // do not use lower PWM as motor will not move anyway
        PWM_MAX); 

// snprintf(buffer, sizeof(buffer), "%d;%d;%d", p, pid_i, pwm);
// TEST_MESSAGE(buffer);

    if (abs(current - pid_prev_rotation) < ROTATION_TOLERANCE)
    {
        pid_i += p;  //no move since last frame
    }
    else
    {
        pid_i = 0;
    }
    pid_prev_rotation = current;

    if (abs(p) < ROTATION_TOLERANCE)
    {
        pwm = 0;
        pid_prev_rotation = 0;
        pid_i = 0;
    }

    // setMotor
    digitalWrite(cfg.rotation_motor.in1, sign1(effort));
    digitalWrite(cfg.rotation_motor.in2, sign2(effort));
    analogWrite(cfg.rotation_motor.ena, pwm);

}