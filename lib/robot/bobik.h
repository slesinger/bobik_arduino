#ifndef bobik_h
#define bobik_h

#include "robot_config_types.h"
#include "caster.h"

class Bobik
{
public:
    Base_t cfg;

    Caster *caster_fl;
    Caster *caster_fr;
    Caster *caster_r;

    Bobik();

private:
    void read_config() {
        #include "robot_config.h"
    }
};

#endif
