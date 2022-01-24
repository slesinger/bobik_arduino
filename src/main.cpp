#include <Arduino.h>
#include <stdint.h>
#include "main.h"
#include "protocol.h"
#include "protocol_types.h"

#include "event_handlers/BobikCasters.h"

#include "sensors/AS5048A.h" //absolute rotation encoder
// #include "sensors/hx711.h"
// #include "sensors/mpu9150.h" //IMU GY-9150

MsgCasterJointStates_t caster_joint_states;
AS5048A fl_caster_rotation_joint(10, 2592); //SPI cable select pin, zero angle value
AS5048A fr_caster_rotation_joint(11, 0); //SPI cable select pin, zero angle value
AS5048A  r_caster_rotation_joint(12, -768); //SPI cable select pin, zero angle value
// hx711 loadcell_upper_arm_lift_joint = hx711(LOADCELL_UPPER_ARM_LIFT_JOINT, 4, 5);
// mpu9150 base_mpu = mpu9150(); //I2C, including Wire.init()


caster_settings_t caster_settings;
BobikCasters casters_handler;


void setup() {
  pinMode(13, OUTPUT);  // for debugging
  protocol_init();
  fl_caster_rotation_joint.init();
  fr_caster_rotation_joint.init();
  r_caster_rotation_joint.init();

  caster_settings.fl_rot = {32, 34, 44};
  caster_settings.fl_drive = {28, 30, 4};
  caster_settings.fr_rot = {40, 42, 45};
  caster_settings.fr_drive = {36, 38, 5};
  caster_settings.r_rot = {41, 43, 46};
  caster_settings.r_drive = {37, 39, 6};
  casters_handler.init(&caster_settings);
  serial_event_message_subscribe(DRIVE_CMD, &casters_handler);

}


void loop() {
  // Loop start
  unsigned long loop_start = millis();
  emit_loop_start();


  // Casters sensors
  caster_joint_states.fl_caster_rotation_joint = (int16_t)fl_caster_rotation_joint.getRotation();
  caster_joint_states.fr_caster_rotation_joint = fr_caster_rotation_joint.getRotation();
  caster_joint_states.r_caster_rotation_joint = r_caster_rotation_joint.getRotation();
  emit_caster_joint_states(&caster_joint_states);

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
