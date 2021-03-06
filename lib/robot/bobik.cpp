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

void Bobik::loop_start()
{
    caster_fl->loop_start();
    caster_fr->loop_start();
    caster_r->loop_start();
}

void Bobik::execute() {
    caster_fl->execute();
    caster_fr->execute();
    caster_r->execute();
}
