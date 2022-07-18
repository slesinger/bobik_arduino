// #include <unity.h>
#include "caster.h"
#include "robot_utils.h"
#include <CircularBuffer.h>

#define AVG_SIZE 10 // how many sensor readings to average
#define ROTATION_TOLERANCE 5
#define DRIVE_TOLERANCE 10
#define DRIVE_MAX_DEBT 100
#define DRIVE_MAX_INTDEBT 10 // add overflow of ticks from last frames (cummulative) to all small speed to overcome excitation energy for motor. It is kept small; set to 0 if robot moves
#define PWM_MAX 255

Caster::Caster(Caster_t caster_cfg)
{
    // initialize class variables
    rotation_target = 0;
    pid_prev_rotation = 0;
    pid_i_rotation = 0;
    driveStoppedDueToRotation = false;
    drive_sensor_ticks_current_counter = 0;
    drive_sensor_tick_last_update_ms = 0;
    pid_i_drive = 0;
    pwm_drive_prev = 0;

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
    digitalWrite(cfg.rotation_motor.in1, RobotUtils::sign1(pwm));
    digitalWrite(cfg.rotation_motor.in2, RobotUtils::sign2(pwm));
    analogWrite(cfg.rotation_motor.ena, abs(pwm));
    delay(time_to_rotate);
    pwm = 0;
    digitalWrite(cfg.rotation_motor.in1, RobotUtils::sign1(pwm));
    digitalWrite(cfg.rotation_motor.in2, RobotUtils::sign2(pwm));
    analogWrite(cfg.rotation_motor.ena, abs(pwm));
    delay(1000);
    pwm = -PWM_MAX;
    digitalWrite(cfg.rotation_motor.in1, RobotUtils::sign1(pwm));
    digitalWrite(cfg.rotation_motor.in2, RobotUtils::sign2(pwm));
    analogWrite(cfg.rotation_motor.ena, abs(pwm));
    delay(time_to_rotate);
    pwm = 0;
    digitalWrite(cfg.rotation_motor.in1, RobotUtils::sign1(pwm));
    digitalWrite(cfg.rotation_motor.in2, RobotUtils::sign2(pwm));
    analogWrite(cfg.rotation_motor.ena, abs(pwm));
    delay(1000);
}

// This is executed at beginning of loop.
// Account for ticks realised and tickes in debt
int16_t Caster::getDriveTicksRealized()
{
    return drive_sensor_ticks_last_frame;
}

float Caster::getDriveTicksDebt()
{
    return 0.0;
    if (drive_debt_queue.size() < 5)
    {
        return 0.0;
    }
    using index_t = decltype(drive_debt_queue)::index_t;
    int16_t sum = 0;
    for (index_t i = 0; i < drive_debt_queue.size(); i++)
    {
        sum += drive_debt_queue[i];
    }
    return (float)sum  / (float)drive_debt_queue.size();
}

void Caster::drive_sensor_tick()
{
    int val = digitalRead(cfg.drive_sensor_pin);
    if (val != last_drive_sensor_val)
    {
        drive_sensor_ticks_current_counter++;
        last_drive_sensor_val = val;
    }
}

void Caster::setDriveTarget(int16_t drive_ticks, bool stoppedFlag)
{
    driveStoppedDueToRotation = stoppedFlag;
    drive_current_frame_required_ticks = drive_ticks;
}

void Caster::pingDriveMotor()
{
    // TEST_MESSAGE("DRIVE");
    int time_to_rotate = 2000;
    int pwm = PWM_MAX;
    digitalWrite(cfg.drive_motor.in1, RobotUtils::sign1(pwm));
    digitalWrite(cfg.drive_motor.in2, RobotUtils::sign2(pwm));
    analogWrite(cfg.drive_motor.ena, abs(pwm));
    delay(time_to_rotate);
    pwm = 0;
    digitalWrite(cfg.drive_motor.in1, RobotUtils::sign1(pwm));
    digitalWrite(cfg.drive_motor.in2, RobotUtils::sign2(pwm));
    analogWrite(cfg.drive_motor.ena, abs(pwm));
    delay(1000);
    pwm = -PWM_MAX;
    digitalWrite(cfg.drive_motor.in1, RobotUtils::sign1(pwm));
    digitalWrite(cfg.drive_motor.in2, RobotUtils::sign2(pwm));
    analogWrite(cfg.drive_motor.ena, abs(pwm));
    delay(time_to_rotate);
    pwm = 0;
    digitalWrite(cfg.drive_motor.in1, RobotUtils::sign1(pwm));
    digitalWrite(cfg.drive_motor.in2, RobotUtils::sign2(pwm));
    analogWrite(cfg.drive_motor.ena, abs(pwm));
    delay(1000);
}

void Caster::loop_start()
{
    drive_sensor_ticks_last_frame = last_frame_ticks_dir * drive_sensor_ticks_current_counter;
    drive_sensor_ticks_current_counter = 0; // make clear cut for cummulated ticks for past frame. Start new frame since now
    drive_debt_queue.push(drive_current_frame_required_ticks - drive_sensor_ticks_last_frame);
}

void Caster::execute()
{
    // char buffer [128];
    // Rtotation PID controller
    int16_t current = this->getRotation();
    int16_t p = rotation_target - current;
    long effort = p + (pid_i_rotation / 4);
    // Full speed rotation changes 145 rotation unit per 50ms
    int pwm_rotation = RobotUtils::map_cut(abs(effort),
                                           0,   // ROTATION_TOLERANCE, // do not move if close enough to target [rot units]
                                           600, // if higher than that full thrust
                                           60,  // do not use lower PWM as motor will not move anyway
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

    if (abs(p) < ROTATION_TOLERANCE + 15)
    {
        pwm_rotation = 0;
        pid_prev_rotation = 0;
        pid_i_rotation = 0;
    }
    // set Rotation Motor
    digitalWrite(cfg.rotation_motor.in1, RobotUtils::sign1(effort));
    digitalWrite(cfg.rotation_motor.in2, RobotUtils::sign2(effort));
    analogWrite(cfg.rotation_motor.ena, pwm_rotation);

    // Drive PID controller
    long effort_drive = drive_current_frame_required_ticks;// + pid_i_drive;                                                 // add debt from last frame if robot was not moving yet had to, slowly
    effort_drive = (abs(effort_drive) > DRIVE_MAX_DEBT) ? DRIVE_MAX_DEBT * RobotUtils::sign(effort_drive) : effort_drive; // do not cummulate debt too much
    // snprintf(buffer, sizeof(buffer), "%d;%d;%d", drive_target, effort_drive, drive_current_frame_required_ticks);
    // TEST_MESSAGE(buffer);

    // cummulate integral partf PID of no move yet shall be
    if ((drive_sensor_ticks_last_frame < 1) && (driveStoppedDueToRotation == false))
    {
        pid_i_drive += drive_current_frame_required_ticks;                                                                 // no move since last frame
        pid_i_drive = (pid_i_drive > DRIVE_MAX_INTDEBT) ? DRIVE_MAX_INTDEBT * RobotUtils::sign(pid_i_drive) : pid_i_drive; // do not cummulate integration too much
    }
    else
    {
        pid_i_drive = 0;
    }

    int16_t pwm_drive = RobotUtils::map_cut(abs(effort_drive), // 12 ticks per frame is full speed
                                            0,                 // do not move if close enough to target [ticks]
                                            14,                // if higher than that full thrust [ticks]
                                            0,                 // do not use lower PWM as motor will not move anyway
                                            PWM_MAX);          // set Drive Motor

    // Smooth pwm start (not stop/break) if start is too abrupt
    if ((pwm_drive - pwm_drive_prev) > 0)
    {
        pwm_drive = round(pwm_drive_prev + (float)(pwm_drive - pwm_drive_prev) / 4.0); // modify previous value by 25% only
    }
    pwm_drive_prev = pwm_drive;

    // make a clear stop if within target
    if ((drive_current_frame_required_ticks == 0) && (abs(effort_drive) < DRIVE_TOLERANCE))
    {
        effort_drive = 0; // expected no move and robot within tolerance from target -> stop explicitly
        pid_i_drive = 0;
        pwm_drive = 0;
        pwm_drive_prev = 0;
    }

    if (pwm_drive == 0)
    {
        digitalWrite(cfg.drive_motor.in1, LOW);
        digitalWrite(cfg.drive_motor.in2, LOW);
        analogWrite(cfg.drive_motor.ena, 200);
    }
    else
    {
        digitalWrite(cfg.drive_motor.in1, RobotUtils::sign1(effort_drive));
        digitalWrite(cfg.drive_motor.in2, RobotUtils::sign2(effort_drive));
        analogWrite(cfg.drive_motor.ena, abs(pwm_drive));          // abs() is not needed here, just safety
    }

    // debug_int = effort_drive;
    last_frame_ticks_dir = RobotUtils::signZero(effort_drive); // will be used next frame to determine ticks to add or sub

    clean_afer_execute();
}

void Caster::clean_afer_execute()
{
    drive_current_frame_required_ticks = 0;
}

void Caster::stopMotors()
{
    drive_debt_queue.clear();
    drive_current_frame_required_ticks = 0;
    pid_i_drive = 0;
    digitalWrite(cfg.drive_motor.in1, LOW);
    digitalWrite(cfg.drive_motor.in2, LOW);
    analogWrite(cfg.drive_motor.ena, 255);

    digitalWrite(cfg.rotation_motor.in1, LOW);
    digitalWrite(cfg.rotation_motor.in2, LOW);
    analogWrite(cfg.rotation_motor.ena, 255);
}
