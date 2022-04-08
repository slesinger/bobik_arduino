#include <math.h>
#include <Arduino.h>
#include "robot_utils.h"
#include "bobik.h"
#include "lidar.h"
// #include <unity.h>

Caster *Bobik::caster_fl;  //must be here because it is static
Caster *Bobik::caster_fr;
Caster *Bobik::caster_r;

Bobik::Bobik()
{
    driveStoppedDueToRotation = false;
    read_config();

    // Casters
    caster_fl = new Caster(cfg.caster_fl);
    caster_fr = new Caster(cfg.caster_fr);
    caster_r  = new Caster(cfg.caster_r);

    // Caster IR ticks reading from drive wheels. Set timer1 interrupt at 1.125kHz. AttachInterrupt on pins does give good results. Equichrono sampling works fine.
    cli(); //stop interrupts
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    OCR1A = 200; // 200~1250Hz; 124 = (16*10^6) / (2000*64) - 1 (must be <256)
    TCCR1B |= (1 << WGM12); // turn on CTC mode
    // Set CS01 and CS00 bits for 64 prescaler
    TCCR1B |= (1 << CS01) | (1 << CS00);
    TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
    sei(); //allow interrupts

    // Lidar
    lidar = new Lidar(cfg.lidar);

}

#include "base_move.h"

void Bobik::execute() {
    // rotation_actual = caster_fl->getRotation();
    // rotation_actual = caster_fr->getRotation();
    // rotation_actual = caster_r->getRotation();

    // ticks_actual_fl += caster_fl->getDriveTicks();
    // ticks_actual_fr += caster_fr->getDriveTicks();
    // ticks_actual_r  += caster_r->getDriveTicks();

    caster_fl->execute();
    caster_fr->execute();
    caster_r->execute();

}


ISR(TIMER1_COMPA_vect)
{
  Bobik::caster_fl->drive_sensor_tick();
  Bobik::caster_fr->drive_sensor_tick();
  Bobik::caster_r->drive_sensor_tick();
}
