// https://docs.platformio.org/en/latest/plus/unit-testing.html#unit-testing

#include <Arduino.h>
#include <unity.h> //https://github.com/ThrowTheSwitch/Unity
#include <bobik.h>
#include "robot_utils.h"
#include <mpu9150.h> //IMU GY-9150
#include <protocol.h>

// More config options: https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU9150/Examples/MPU9150_DMP9/MPU9150_DMP9.ino

/*
 * Flags
 */

#define TEST_MOTORS
// #define SERIAL_OUTPUT // if enabled run "pio test", Ctrl+C, "pio device monitor -b 500000" to see serial output

/*
 * Enable required test suites here:
 */
#define TEST_XXX "xXx"

mpu9150 base_mpu = mpu9150();


void test_mpu(void)
{
    MsgIMU9DOF_t imu_msg;
    for (int i = 0; i < 100; i++)
    {
        delay(55);
        base_mpu.readSensorQAG(&imu_msg);
        Serial.print(imu_msg.qw);
        Serial.print("\t");
        Serial.print(imu_msg.qx);
        Serial.print("\t");
        Serial.print(imu_msg.qy);
        Serial.print("\t");
        Serial.print(imu_msg.qz);
        Serial.print("\t - \t");

        Serial.print(imu_msg.gx);
        Serial.print("\t");
        Serial.print(imu_msg.gy);
        Serial.print("\t");
        Serial.print(imu_msg.gz);
        Serial.print("\t - \t");

        Serial.print(imu_msg.ax);
        Serial.print("\t");
        Serial.print(imu_msg.ay);
        Serial.print("\t");
        Serial.println(imu_msg.az);
    }
}

void setup()
{
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    Serial.begin(115200);
    delay(1000);
    base_mpu.init();

#ifndef SERIAL_OUTPUT
    UNITY_BEGIN();
    TEST_MESSAGE("START UNIT TESTS");

#ifdef TEST_XXX
    TEST_MESSAGE(TEST_XXX);
    RUN_TEST(test_mpu);
#endif

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
