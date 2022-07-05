#include "mpu9150.h"
// #include <protocol.h>

//https://cdn.sparkfun.com/datasheets/Sensors/IMU/MPU-9150-Register-Map.pdf

MPU9150 accelGyroMag = MPU9150(0x68);
boolean conn_ok = false;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
MsgIMU9DOF_t imu;

mpu9150::mpu9150()
{
  Wire.begin();
  accelGyroMag.initialize();
  conn_ok = accelGyroMag.testConnection();
}

mpu9150::~mpu9150()
{
}

void mpu9150::readSensor9(MsgIMU9DOF_t *msg)
{
  if (conn_ok) {
    accelGyroMag.getMotion9(&(imu.ax), &(imu.ay), &(imu.az), &(imu.gx), &(imu.gy), &(imu.gz), &(imu.mx), &(imu.my), &(imu.mz));
  }
}
