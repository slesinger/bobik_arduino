#include "hx711.h"
#define ARDUINO 10813
#include <protocol.h>
#include "HX711.h"

HX711 scale;

hx711::hx711(unsigned char ptype, int dout_pin, int sck_pin)
{
  protocol_type = ptype;
  scale.begin(dout_pin, sck_pin);
}

hx711::~hx711()
{
}

void hx711::run()
{
  if (scale.wait_ready_retry(1, 0))
  {
    emit1(protocol_type, scale.read());
  }
}
