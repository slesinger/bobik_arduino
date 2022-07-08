#ifndef BMPU9150_H
#define BMPU9150_H

#include <protocol.h>

class mpu9150
{
public:
  mpu9150();

  void init();
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