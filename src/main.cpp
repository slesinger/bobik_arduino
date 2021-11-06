#include <Arduino.h>
#include <stdint.h>
#include "protocol.h"
#include "protocol_types.h"

// #include "modules/hx711.h"
#include "modules/AS5048A.h" //absolute rotation encoder
// #include "modules/mpu9150.h" //IMU GY-9150

#define WAIT_GRANULARITY 1 //[ms]
uint8_t log_buf[8];

// hx711 loadcell_upper_arm_lift_joint = hx711(LOADCELL_UPPER_ARM_LIFT_JOINT, 4, 5);
MsgCasterJointStates_t caster_joint_states;
AS5048A fl_caster_rotation_joint(10, 2592); //SPI cable select pin, zero angle value
AS5048A fr_caster_rotation_joint(11, 0); //SPI cable select pin, zero angle value
AS5048A  r_caster_rotation_joint(12, -768); //SPI cable select pin, zero angle value
// mpu9150 base_mpu = mpu9150(); //I2C, including Wire.init()
const unsigned int MAIN_LOOP_FREQ_MS = 50; //[ms] = 1000[ms]/freq[Hz] target is 10ms


void setup() {
  protocol_init();
  fl_caster_rotation_joint.init();
  fr_caster_rotation_joint.init();
  r_caster_rotation_joint.init();

}


void loop() {
  // Loop start
  unsigned long loop_start = millis();
  emit_loop_start();


  // Casters
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

  // Receive from Driver while keeping loop freq
  dispatch_until(loop_start + MAIN_LOOP_FREQ_MS, WAIT_GRANULARITY, log_buf);
}
