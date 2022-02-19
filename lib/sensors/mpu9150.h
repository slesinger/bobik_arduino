#ifndef MPU9250_H
#define MPU9250_H

#include "MPU9150.h"
#include "Wire.h"

class mpu9150
{
private:
  

public:
  mpu9150();
  virtual ~mpu9150();

  void run();
};

#endif