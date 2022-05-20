/*
https://www.geeetech.com/wiki/index.php/L298N_Motor_Driver_Board
*/
#include <Arduino.h>
#include <stdint.h>
#include "BobikCasters.h"
#include "protocol_types.h"
#include "robot_utils.h"

MsgCmdVel_t msg_cmd_vel;

BobikCasters::BobikCasters(Bobik *bobik)
{
    robot = bobik;
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
    memcpy(&msg_cmd_vel, data, sizeof(msg_cmd_vel));
    float x = msg_cmd_vel.linear_x / FLOAT_INT16_PRECISION;
    float y = msg_cmd_vel.linear_y / FLOAT_INT16_PRECISION;
    float gamma = msg_cmd_vel.rotation / FLOAT_INT16_PRECISION;
    robot->setCmdVel(x, y, gamma);
    log_buf[0] = (int8_t)msg_cmd_vel.linear_x;
    log_buf[1] = (int8_t)(msg_cmd_vel.linear_x >> 8);
    log_buf[2] = (int8_t)msg_cmd_vel.linear_y;
    log_buf[3] = (int8_t)(msg_cmd_vel.linear_y >> 8);

}