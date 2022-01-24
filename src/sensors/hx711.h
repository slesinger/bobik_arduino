#ifndef hx711_h
#define hx711_h

class hx711
{
private:
  unsigned char protocol_type;

public:
  hx711(unsigned char protocol_type, int dout_pin, int sck_pin);
  virtual ~hx711();

  void run();
};

#endif