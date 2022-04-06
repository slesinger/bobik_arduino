#ifndef bobikcasters_h
#define bobikcasters_h

#include <stdint.h>
#include <ievent_handler.h>
#include <bobik.h>

struct motor_pins_t
{
    uint8_t in1;
    uint8_t in2;
    uint8_t ena;
};

struct caster_settings_t
{
    int id;
    struct motor_pins_t fl_rot;
    struct motor_pins_t fl_drive;
    struct motor_pins_t fr_rot;
    struct motor_pins_t fr_drive;
    struct motor_pins_t  r_rot;
    struct motor_pins_t  r_drive;
};

class BobikCasters: public IEventHandler 
{
    public:
        BobikCasters(Bobik *bobik);
        void init(caster_settings_t *settings);
        void serial_message_handler(unsigned char* data, uint8_t *log_buf);

    private:
        struct caster_settings_t *caster_settings;
        Bobik *robot;

};

#endif
