#ifndef MPU9250_H
#define MPU9250_H

// #include "MPU9150.h"
// #include "Wire.h"
#include <protocol.h>

class mpu9150
{
public:
  mpu9150();
  ~mpu9150();

  /**
   * @brief read values from accelerometer, gyro and magnetometer
   * @arg msg
   *
   * @return 0 for success. Values are updated in message.
   */
  uint8_t readSensorQAG(MsgIMU9DOF_t *msg);

private:
};

#endif