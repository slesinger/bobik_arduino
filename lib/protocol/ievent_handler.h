#ifndef ieventhandler_h
#define ieventhandler_h

#include <stdint.h>
#include <Arduino.h>

class IEventHandler
{
    public:
        virtual void serial_message_handler(unsigned char* data) {
            serial_message_handler(data, NULL);
        }
        virtual void serial_message_handler(unsigned char* data, uint8_t *log_buf);
};

#endif
