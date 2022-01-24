/*
https://www.geeetech.com/wiki/index.php/L298N_Motor_Driver_Board
*/
#include <Arduino.h>
#include <stdint.h>
#include "BobikCasters.h"
#include "protocol_types.h"

MsgRawCmdVel_t raw_cmd_vel;

int sign1(int16_t x) {
    return (x > 0) ? HIGH : LOW;
}
int sign2(int16_t x) {
    return (x < 0) ? HIGH : LOW;
}

BobikCasters::BobikCasters()
{
}

void BobikCasters::init(caster_settings_t *settings)
{
    caster_settings = settings;
    pinMode(caster_settings->fl_rot.in1, OUTPUT);
    pinMode(caster_settings->fl_rot.in2, OUTPUT);
    pinMode(caster_settings->fl_rot.ena, OUTPUT);
    pinMode(caster_settings->fl_drive.in1, OUTPUT);
    pinMode(caster_settings->fl_drive.in2, OUTPUT);
    pinMode(caster_settings->fl_drive.ena, OUTPUT);
    pinMode(caster_settings->fr_rot.in1, OUTPUT);
    pinMode(caster_settings->fr_rot.in2, OUTPUT);
    pinMode(caster_settings->fr_rot.ena, OUTPUT);
    pinMode(caster_settings->fr_drive.in1, OUTPUT);
    pinMode(caster_settings->fr_drive.in2, OUTPUT);
    pinMode(caster_settings->fr_drive.ena, OUTPUT);
    pinMode(caster_settings->r_rot.in1, OUTPUT);
    pinMode(caster_settings->r_rot.in2, OUTPUT);
    pinMode(caster_settings->r_rot.ena, OUTPUT);
    pinMode(caster_settings->r_drive.in1, OUTPUT);
    pinMode(caster_settings->r_drive.in2, OUTPUT);
    pinMode(caster_settings->r_drive.ena, OUTPUT);
}

void BobikCasters::serial_message_handler(unsigned char* data, uint8_t *log_buf)
{
    memcpy(&raw_cmd_vel, data, sizeof(raw_cmd_vel));

    int16_t desired_rot = 0; //raw_cmd_vel.fl_caster_rotation;
    int16_t desired_drive = raw_cmd_vel.fl_caster_drive;
    log_buf[0] = (int8_t)raw_cmd_vel.fl_caster_drive;
    log_buf[1] = (int8_t)raw_cmd_vel.fl_caster_rotation;
    log_buf[2] = caster_settings->fl_drive.in1;
    log_buf[3] = caster_settings->fl_drive.in2;

    digitalWrite(caster_settings->fl_rot.in1, sign1(desired_rot));
    digitalWrite(caster_settings->fl_rot.in2, sign2(desired_rot));
    analogWrite(caster_settings->fl_rot.ena, desired_rot);
    digitalWrite(caster_settings->fl_drive.in1, sign1(desired_drive));
    digitalWrite(caster_settings->fl_drive.in2, sign2(desired_drive));
    analogWrite(caster_settings->fl_drive.ena, desired_drive);

    digitalWrite(caster_settings->fr_rot.in1, sign1(desired_rot));
    digitalWrite(caster_settings->fr_rot.in2, sign2(desired_rot));
    analogWrite(caster_settings->fr_rot.ena, desired_rot);
    digitalWrite(caster_settings->fr_drive.in1, sign1(desired_drive));
    digitalWrite(caster_settings->fr_drive.in2, sign2(desired_drive));
    analogWrite(caster_settings->fr_drive.ena, desired_drive);

    digitalWrite(caster_settings->r_rot.in1, sign1(desired_rot));
    digitalWrite(caster_settings->r_rot.in2, sign2(desired_rot));
    analogWrite(caster_settings->r_rot.ena, desired_rot);
    digitalWrite(caster_settings->r_drive.in1, sign1(desired_drive));
    digitalWrite(caster_settings->r_drive.in2, sign2(desired_drive));
    analogWrite(caster_settings->r_drive.ena, desired_drive);

}