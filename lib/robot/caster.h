#ifndef caster_h
#define caster_h

#include <AS5048A.h> //absolute rotation encoder
#include "robot_config_types.h"
#include "component.h"

class Caster : public Component
{
public:

    Caster(Caster_t caster_cfg);
    int getRotation();
    void execute();

private:
    Caster_t cfg;
    AS5048A *rotation_sensor;

};

#endif
