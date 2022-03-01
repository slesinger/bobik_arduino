#include "caster.h"
#include "utils.h"
#include <unity.h>

#define AVG_SIZE 7 // how many sensor readings to average
#define ROTATION_TOLERANCE 30
#define DRIVE_TOLERANCE 20
#define PWM_MAX 255

Caster *Caster::caster_fl;
Caster *Caster::caster_fr;
Caster *Caster::caster_r;

Caster::Caster(Caster_t caster_cfg)
{
    // initialize class variables
    rotation_target = 0;
    pid_prev_rotation = 0;
    pid_i_rotation = 0;
    drive_sensor_ticks = 0;
    drive_sensor_tick_last_update_ms = 0;
    drive_target = 0;
    pid_i_drive = 0;

    cfg = caster_cfg;
    rotation_sensor = new AS5048A(cfg.rotation_sensor.spi_cs_pin, cfg.rotation_sensor.zero_position); // SPI cable select pin, zero angle value
    rotation_sensor->init();

    // read first values which are inaccurate
    for (int i = 0; i < 20; i++)
    {
        getRotation();
    }

    // Rotation motor
    pinMode(cfg.rotation_motor.in1, OUTPUT);
    pinMode(cfg.rotation_motor.in2, OUTPUT);
    pinMode(cfg.rotation_motor.ena, OUTPUT);

    // Drive motor
    pinMode(cfg.drive_motor.in1, OUTPUT);
    pinMode(cfg.drive_motor.in2, OUTPUT);
    pinMode(cfg.drive_motor.ena, OUTPUT);

    pinMode(cfg.drive_sensor_pin, INPUT);
    digitalWrite(cfg.drive_sensor_pin, HIGH); // set pullup resistor
    switch (cfg.drive_sensor_pin)             // this is needed because only static method can be used with attachInterrupt
    {
    case 20:
        attachInterrupt(digitalPinToInterrupt(cfg.drive_sensor_pin), drive_sensor_interrupt_fl, CHANGE);
        caster_fl = this;
        break;
    case 19:
        attachInterrupt(digitalPinToInterrupt(cfg.drive_sensor_pin), drive_sensor_interrupt_fr, CHANGE);
        caster_fr = this;
        break;
    case 21:
        attachInterrupt(digitalPinToInterrupt(cfg.drive_sensor_pin), drive_sensor_interrupt_r, CHANGE);
        caster_r = this;
        break;

    default:
        break;
    }
}

int16_t Caster::getRotation()
{
    long sum = 0;
    for (int i = 0; i < AVG_SIZE; i++)
    {
        uint16_t raw = rotation_sensor->getRawRotation();
        int16_t rotation = raw - cfg.rotation_sensor.zero_position;
        if (rotation > 8191)
            rotation = -((0x3FFF) - rotation); // more than -180
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
    int time_to_rotate = 200;
    int pwm = PWM_MAX;
    digitalWrite(cfg.rotation_motor.in1, sign1(pwm));
    digitalWrite(cfg.rotation_motor.in2, sign2(pwm));
    analogWrite(cfg.rotation_motor.ena, abs(pwm));
    delay(time_to_rotate);
    pwm = 0;
    digitalWrite(cfg.rotation_motor.in1, sign1(pwm));
    digitalWrite(cfg.rotation_motor.in2, sign2(pwm));
    analogWrite(cfg.rotation_motor.ena, abs(pwm));
    delay(1000);
    pwm = -PWM_MAX;
    digitalWrite(cfg.rotation_motor.in1, sign1(pwm));
    digitalWrite(cfg.rotation_motor.in2, sign2(pwm));
    analogWrite(cfg.rotation_motor.ena, abs(pwm));
    delay(time_to_rotate);
    pwm = 0;
    digitalWrite(cfg.rotation_motor.in1, sign1(pwm));
    digitalWrite(cfg.rotation_motor.in2, sign2(pwm));
    analogWrite(cfg.rotation_motor.ena, abs(pwm));
    delay(1000);
}

int16_t Caster::getDriveTicks()
{
    return drive_sensor_ticks;
}

void Caster::inc_drive_sensor_tick()
{
    unsigned long now_ms = millis();
    if ((drive_sensor_tick_last_update_ms + 1 ) <= now_ms)
    {
        drive_sensor_ticks++;
        drive_sensor_tick_last_update_ms = now_ms;
    }
}

void Caster::drive_sensor_interrupt_fl()
{
    caster_fl->inc_drive_sensor_tick();
}
void Caster::drive_sensor_interrupt_fr()
{
    caster_fr->inc_drive_sensor_tick();
}
void Caster::drive_sensor_interrupt_r()
{
    caster_r->inc_drive_sensor_tick();
}

void Caster::pingDriveMotor()
{
    TEST_MESSAGE("DRIVE");
    int time_to_rotate = 2000;
    int pwm = PWM_MAX;
    digitalWrite(cfg.drive_motor.in1, sign1(pwm));
    digitalWrite(cfg.drive_motor.in2, sign2(pwm));
    analogWrite(cfg.drive_motor.ena, abs(pwm));
    delay(time_to_rotate);
    pwm = 0;
    digitalWrite(cfg.drive_motor.in1, sign1(pwm));
    digitalWrite(cfg.drive_motor.in2, sign2(pwm));
    analogWrite(cfg.drive_motor.ena, abs(pwm));
    delay(1000);
    pwm = -PWM_MAX;
    digitalWrite(cfg.drive_motor.in1, sign1(pwm));
    digitalWrite(cfg.drive_motor.in2, sign2(pwm));
    analogWrite(cfg.drive_motor.ena, abs(pwm));
    delay(time_to_rotate);
    pwm = 0;
    digitalWrite(cfg.drive_motor.in1, sign1(pwm));
    digitalWrite(cfg.drive_motor.in2, sign2(pwm));
    analogWrite(cfg.drive_motor.ena, abs(pwm));
    delay(1000);
}

void Caster::setDriveTarget(int16_t drive_ticks)
{
    drive_target += drive_ticks; // add requirement for new frame to current target
}

void Caster::execute()
{
    uint16_t last_frame_ticks = drive_sensor_ticks;
    drive_sensor_ticks = 0;  //make clear cut for cummulated ticks for past frame. Start new frame since now

    char buffer [128];
    // Rtotation PID controller
    int16_t current = this->getRotation();
    int16_t p = rotation_target - current;
    long effort = p + (pid_i_rotation / 2);
    // Full speed rotation changes 145 rotation unit per 50ms
    int pwm_rotation = map_cut(abs(effort),
                      ROTATION_TOLERANCE, // do not move if close enough to target [rot units]
                      600,                // if higher than that full thrust
                      100,                // do not use lower PWM as motor will not move anyway
                      PWM_MAX);

    // snprintf(buffer, sizeof(buffer), "%d;%d;%d", p, pid_i_rotation, pwm);
    // TEST_MESSAGE(buffer);

    if (abs(current - pid_prev_rotation) < ROTATION_TOLERANCE)
    {
        pid_i_rotation += p; // no move since last frame
    }
    else
    {
        pid_i_rotation = 0;
    }
    pid_prev_rotation = current;

    if (abs(p) < ROTATION_TOLERANCE)
    {
        pwm_rotation = 0;
        pid_prev_rotation = 0;
        pid_i_rotation = 0;
    }

    // set Rotation Motor
    digitalWrite(cfg.rotation_motor.in1, sign1(effort));
    digitalWrite(cfg.rotation_motor.in2, sign2(effort));
    analogWrite(cfg.rotation_motor.ena, pwm_rotation);


    // Rotation PID controller
    drive_target -= last_frame_ticks * last_frame_ticks_dir; // subtract what has been driven out from the target //PID P
    long effort_drive = drive_target + (pid_i_drive / 2);
    int pwm_drive = map_cut(abs(effort_drive),
                      DRIVE_TOLERANCE, // do not move if close enough to target [ticks]
                      50,              // if higher than that full thrust [ticks]
                      150,             // do not use lower PWM as motor will not move anyway
                      PWM_MAX);        // set Drive Motor
    digitalWrite(cfg.drive_motor.in1, sign1(drive_target));
    digitalWrite(cfg.drive_motor.in2, sign2(drive_target));
    analogWrite(cfg.drive_motor.ena, abs(pwm_drive));
    last_frame_ticks_dir = sign(drive_target);  // will be used next frame to determine ticks to add or sub
    snprintf(buffer, sizeof(buffer), "%d;%d;%d", drive_target, pid_i_drive, pwm_drive);
    TEST_MESSAGE(buffer);

}

void Caster::stopMotors()
{
    analogWrite(cfg.rotation_motor.ena, 0);
}

void Caster::stopAllCastersMotors()
{
    caster_fl->stopMotors();
    caster_fr->stopMotors();
    caster_r->stopMotors();
}