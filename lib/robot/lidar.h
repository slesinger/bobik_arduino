#ifndef lidar_h
#define lidar_h

#include "robot_config_types.h"

#define LIDAR_PWM 180

class Lidar
{
private:
public:
    Lidar(Lidar_t lidar_cfg);
    ~Lidar();
};

#endif
