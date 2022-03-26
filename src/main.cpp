#include <Arduino.h>
#include <stdint.h>
#include "main.h"
#include <bobik.h>
#include <protocol.h>
#include <protocol_types.h>

#include <BobikCasters.h>
#include "robot_utils.h"

// #include <hx711.h>
// #include <mpu9150.h> //IMU GY-9150

MsgCasterJointStates_t caster_joint_states;
// hx711 loadcell_upper_arm_lift_joint = hx711(LOADCELL_UPPER_ARM_LIFT_JOINT, 4, 5);
// mpu9150 base_mpu = mpu9150(); //I2C, including Wire.init()

caster_settings_t caster_settings;
BobikCasters casters_handler;

Bobik robot;

void setup()
{
  pinMode(13, OUTPUT); // for debugging
  protocol_init();

  casters_handler.init(&caster_settings);
  serial_event_message_subscribe(DRIVE_CMD, &casters_handler);
}

void loop()
{
  // Loop start
  unsigned long loop_start = millis();
  emit_loop_start();

  // Casters sensors
  // caster_joint_states.fl_caster_rotation_joint = (int16_t)fl_caster_rotation_joint.getRotation();
  // caster_joint_states.fr_caster_rotation_joint = fr_caster_rotation_joint.getRotation();
  // caster_joint_states.r_caster_rotation_joint = r_caster_rotation_joint.getRotation();
  // emit_caster_joint_states(&caster_joint_states);

  // MPU
  // base_mpu.run();

  // loadcell_upper_arm_lift_joint.run();
  // uint16_t lap = (uint16_t)(millis() - loop_start);
  // emit4(LOG4, (uint8_t)(lap>>8), (uint8_t)lap, log_buf[2], log_buf[3]); //read takes about ~1ms

  emit4(LOG4, log_buf[0], log_buf[1], log_buf[2], log_buf[3]);

  // Loop end
  emit_loop_end();

  // Receive commands from bobik_driver while keeping loop freq
  dispatch_until(loop_start + MAIN_LOOP_FREQ_MS, WAIT_GRANULARITY, log_buf);
}
