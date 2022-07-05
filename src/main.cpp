#include <Arduino.h>
#include <stdint.h>
#include "main.h"
#include <bobik.h>
#include <protocol.h>
#include <protocol_types.h>
#include "robot_utils.h"
#include "BobikCasters.h"
#include <mpu9150.h> //IMU GY-9150
// #include <hx711.h>

#define DEBUG_SERIAL


Bobik *robot = new Bobik();

// Define messages sent to driver
MsgCasterJointStates_t caster_joint_states;
MsgIMU9DOF_t imu_msg;
// hx711 loadcell_upper_arm_lift_joint = hx711(LOADCELL_UPPER_ARM_LIFT_JOINT, 4, 5);

// Create robot components
caster_settings_t caster_settings;
BobikCasters *casters_handler = new BobikCasters(robot);
mpu9150 base_mpu = mpu9150();


void setup()
{
  pinMode(13, OUTPUT); // for debugging
  protocol_init();

  casters_handler->init(&caster_settings);

  serial_event_message_subscribe(DRIVE_CMD, casters_handler);

#ifdef DEBUG_SERIAL
  Serial.begin(9600);
#endif
}

void loop()
{
  // Loop start
  unsigned long loop_start = millis();
  emit_loop_start();
  robot->loop_start();

  // Casters sensors
  caster_joint_states.fl_caster_rotation_joint = robot->caster_fl->getRotation();
  caster_joint_states.fr_caster_rotation_joint = robot->caster_fr->getRotation();
  caster_joint_states.r_caster_rotation_joint = robot->caster_r->getRotation();
  caster_joint_states.fl_caster_drive_joint = robot->caster_fl->getDriveTicksRealized();
  caster_joint_states.fr_caster_drive_joint = robot->caster_fr->getDriveTicksRealized();
  caster_joint_states.r_caster_drive_joint = robot->caster_r->getDriveTicksRealized();
  emit_caster_joint_states(&caster_joint_states);

  // IMU sensor
  base_mpu.readSensor9(&imu_msg);
  emit_IMU9DOF(&imu_msg);

  // loadcell_upper_arm_lift_joint.run();
  // uint16_t lap = (uint16_t)(millis() - loop_start);
  // emit4(LOG4, (uint8_t)(lap>>8), (uint8_t)lap, log_buf[2], log_buf[3]); //read takes about ~1ms

  robot->execute();

  // Send logs to driver
  if (log_buf[0] != 0 || log_buf[1] != 0 || log_buf[2] != 0 || log_buf[3] != 0)
  {
    emit4(LOG4, log_buf[0], log_buf[1], log_buf[2], log_buf[3]);
    log_buf[0] = 0;
    log_buf[1] = 0;
    log_buf[2] = 0;
    log_buf[3] = 0;
  }

  // Loop end
  emit_loop_end();

  // Receive commands from bobik_driver while keeping loop freq
  dispatch_until(loop_start + MAIN_LOOP_FREQ_MS, WAIT_GRANULARITY, log_buf);
}
