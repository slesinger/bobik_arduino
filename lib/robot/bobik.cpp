#include "bobik.h"

Bobik::Bobik()
{
    read_config();

    // Casters
    caster_fl = new Caster(cfg.caster_fl);
    caster_fr = new Caster(cfg.caster_fr);
    caster_r  = new Caster(cfg.caster_r);
}
