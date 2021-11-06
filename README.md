# bobik_arduino
Firmware for main Arduino board
# PlatformIO VS Code
Include this line to C/C++ Configurations:
```
/home/honza/.platformio/packages/**
```

# Important Notes

If serial send and receive does not work in parallel, reset Arduino

# Arduino Pins Description

https://aws1.discourse-cdn.com/arduino/original/4X/4/0/c/40ca0db220e359ad94a4e61e70d0a54406986232.png

```
28 - IN1 drive caster LF (green, 1 stripes, far from pin)
30 - IN2 drive caster LF (blue, 1 stripes, far from pin)
32 - IN3 rotation caster LF (green, 1 stripes, close to pin)
34 - IN4 rotation caster LF (blue, 1 stripes, close to pin)
36 - IN1 drive caster RF (green, 2 stripes, far from pin)
38 - IN2 drive caster RF (blue, 2 stripes, far from pin)
40 - IN3 rotation caster RF (green, 2 stripes, close to pin)
42 - IN4 rotation caster RF (blue, 2 stripes, close to pin)

37 - IN1 drive caster R (green, 3 stripes, far from pin)
39 - IN2 drive caster R (blue, 3 stripes, far from pin)
41 - IN3 rotation caster R (green, 3 stripes, close to pin)
43 - IN4 rotation caster R (blue, 3 stripes, close to pin)

44 - PWM ENA rotation caster LF (yellow, 1 stripe, close to pin)
45 - PWM ENA rotation caster RF (yellow, 2 stripes, close to pin)
46 - PWM ENA rotation caster R (yellow, 3 stripes, close to pin)

```