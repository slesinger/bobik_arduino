#ifndef robot_config_h
#define robot_config_h

/*
*   This code is #included to Bobik::read_config
*/

    // Base


    // Casters
    cfg.caster_fl.rotation_sensor.spi_cs_pin = 10;
    cfg.caster_fl.rotation_sensor.zero_position = 242;
    cfg.caster_fl.rotation_motor = {32, 34, 44};
    cfg.caster_fl.drive_motor = {28, 30, 4};
    cfg.caster_fl.drive_sensor_pin = 20;
    cfg.caster_fr.rotation_sensor.spi_cs_pin = 11;
    cfg.caster_fr.rotation_sensor.zero_position = 416;
    cfg.caster_fr.rotation_motor = {40, 42, 45};
    cfg.caster_fr.drive_motor = {36, 38, 5};
    cfg.caster_fr.drive_sensor_pin = 21;
    cfg.caster_r.rotation_sensor.spi_cs_pin =  12;
    cfg.caster_r.rotation_sensor.zero_position = 386;
    cfg.caster_r.rotation_motor = {41, 43, 46};
    cfg.caster_r.drive_motor = {37, 39, 6};
    cfg.caster_r.drive_sensor_pin = 19;

    // Head

    // Arm


#endif
