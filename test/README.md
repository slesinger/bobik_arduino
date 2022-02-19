# bobik_arduino
Unit tests to prove HW response with output to serial console
# PlatformIO VS Code
Include this line to C/C++ Configurations:
```
/home/honza/.platformio/packages/**
```

# Arduino Pins Description

https://aws1.discourse-cdn.com/arduino/original/4X/4/0/c/40ca0db220e359ad94a4e61e70d0a54406986232.png

See ```bobik_arduino``` README.

# Run Test
```
pio run --target=upload && pio device monitor -b 500000
pio test && pio device monitor -b 500000

```
