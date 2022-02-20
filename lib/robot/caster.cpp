#include "caster.h"

Caster::Caster(Caster_t caster_cfg)
{
    cfg = caster_cfg;
    rotation_sensor = new AS5048A(cfg.rotation_sensor.spi_cs_pin, cfg.rotation_sensor.zero_position); // SPI cable select pin, zero angle value
    rotation_sensor->init();
}

int Caster::getRotation()
{
    uint16_t raw = rotation_sensor->getRawRotation();
    int16_t rotation = raw - cfg.rotation_sensor.zero_position;
	if(rotation > 8191) rotation = -((0x3FFF)-rotation); //more than -180
    return (int)rotation;
}

void Caster::execute()
{
}