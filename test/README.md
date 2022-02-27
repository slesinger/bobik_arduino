# bobik_arduino
Unit tests to prove HW response with output to serial console
# PlatformIO VS Code
Include this line to C/C++ Configurations:
```
/home/honza/.platformio/packages/**
```

# Run Test
```
pio run --target=upload && pio device monitor -b 500000
pio test && pio device monitor -b 500000

```
