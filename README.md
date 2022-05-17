# bobik_arduino
Firmware for main Arduino board
# PlatformIO VS Code
Include this line to C/C++ Configurations as "Include path":
```
/home/honza/.platformio/packages/**
```

# Important Notes

If serial send and receive does not work in parallel, reset Arduino

# Arduino Pins Description

https://aws1.discourse-cdn.com/arduino/original/4X/4/0/c/40ca0db220e359ad94a4e61e70d0a54406986232.png

```
4 - PWM ENA drive caster LF (yellow, 1 stripe, close to pin)
5 - PWM ENA drive caster RF (yellow, 2 stripes, close to pin)
6 - PWM ENA drive caster R (yellow, 3 stripes, close to pin)
9 - PWM XV-11 Lidar motor
10 - neco s AS5048!!!!!
11 - neco s AS5048!!!!!
14 - TX to Jetson UART1 RXD J3A2 65 (green, needs level shifter to 1.8v, /dev/ttyTHS1  (https://elinux.org/Jetson/GPIO#GPIO_on_Jetson_TK1))
15 - RX to Jetson UART1 TXD J3A2 68 (white, needs level shifter to 1.8v)
19 - odom IR tick signal FL
20 - odom IR tick signal FR
21 - odom IR tick signal R

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

# Realtime Architecture

- receive commands from serial > store to memory
- independent cycle of 1kHz
  - handlers that execute on each cycle or on a divider
    - serial send on occurence of data


# Protocol Description

serialize/deserialize
Loose reference to ROS messages. Not using rosserial due to issues durong compilation. Deprecated package.

## Object types
- geometry_msgs/Twist Message, misused also for odometry
- http://wiki.ros.org/std_msgs for status reporting

# Leadync synch datagram for cycle start
$AA 55 - cycle start
timestamp unsigned long (4bytes) - number of milliseconds passed since the Arduino board began running; overflows after approximately 50 days
checksum - _crc8_ccitt_update of payload (http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html)

## Dataframe Structure
$AA - datagram start
<type><subject>   - serialized type of data 4bit, subject who is reporting 4bit
<serialized data> - payload
checksum - _crc8_ccitt_update of payload

> Timestamp is used only at begining of cycle as data are considered realtime, within the 1kHz cycle. Timestamp does not use realtime clock.

### <type><subject> List
```
55 cycle start
1  geometry_msgs/Twist Message
 


```

### <type> Definitions


# Arduino pin setup

10 - AS5048A SPI CS FL
11 - AS5048A SPI CS FR
12 - AS5048A SPI CS R
50 - AS5048A MISO
51 - AS5048A MOSI
52 - AS5048A SPI CLK

## AM5048A
https://ams.com/documents/20143/36005/AS5048_DS000298_4-00.pdf

https://zoodmall.com/cdn-cgi/image/w=500,fit=contain,f=auto/http://luckyretail.com/Uploadfile/201907141/257786/257786-5.jpg

CSn - green
CLK - blue
MOSI- yellow
MISO- white
V5V - red
GND - black
