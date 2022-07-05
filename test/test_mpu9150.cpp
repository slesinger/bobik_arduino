// https://docs.platformio.org/en/latest/plus/unit-testing.html#unit-testing

#include <Arduino.h>
#include <unity.h> //https://github.com/ThrowTheSwitch/Unity
#include <bobik.h>
#include "robot_utils.h"
#include "mpu9150.h"
#include <protocol.h>

/*
 * Flags
 */
#define TEST_MOTORS
// #define SERIAL_OUTPUT // if enabled run "pio test", Ctrl+C, "pio device monitor -b 500000" to see serial output

/*
 * Enable required test suites here:
 */
#define TEST_XXX "xXx"

Bobik robot;
MPU9150 accelGyroMag = MPU9150(0x68);

void test_mpu(void)
{
  MsgIMU9DOF_t imu;
  for (int i = 0; i < 1000; i++)
  {
    accelGyroMag.getMotion9(&(imu.ax), &(imu.ay), &(imu.az), &(imu.gx), &(imu.gy), &(imu.gz), &(imu.mx), &(imu.my), &(imu.mz));
    Serial.print(imu.ax);
    Serial.print('\t');
    Serial.print(imu.ay);
    Serial.print('\t');
    Serial.print(imu.az);
    Serial.println('\t');
  }
}

int f2i(float f)
{
  return round(f * 1000);
}

void setup()
{
  // NOTE!!! Wait for >2 secs
  // if board doesn't support software reset via Serial.DTR/RTS
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Serial.begin(500000);
  accelGyroMag.initialize();
  delay(1000);
#ifndef SERIAL_OUTPUT
  UNITY_BEGIN();
  TEST_MESSAGE("START UNIT TESTS");

#ifdef TEST_XXX
  TEST_MESSAGE(TEST_XXX);
  RUN_TEST(test_mpu);
#endif TEST_XXX

  TEST_MESSAGE("FINISH UNIT TESTS");
  UNITY_END();
#endif // not serial_output

  digitalWrite(13, HIGH);
}

void loop()
{
  TEST_MESSAGE("Pause in loop. Will test agin in 10s.");
  delay(10000);
}
