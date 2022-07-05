#ifndef MPU9250_H
#define MPU9250_H

#include "MPU9150.h"
#include "Wire.h"
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
   * @return Nothing. Values are updated in message.
   */
  void readSensor9(MsgIMU9DOF_t *msg);

private:
};

#endif