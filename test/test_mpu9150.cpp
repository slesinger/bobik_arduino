// https://docs.platformio.org/en/latest/plus/unit-testing.html#unit-testing

#include <Arduino.h>
#include <unity.h> //https://github.com/ThrowTheSwitch/Unity
#include <bobik.h>
#include "robot_utils.h"
#include "MPU9150_9Axis_MotionApps41.h"
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

Bobik robot;
MPU9150 mpu;

uint8_t mpuIntStatus = 0; // holds actual interrupt status byte from MPU
uint8_t devStatus;        // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize = 48; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;       // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];   // FIFO storage buffer

Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
int16_t gyro[3];     // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

void test_mpu(void)
{
    // MsgIMU9DOF_t imu;
    for (int i = 0; i < 1000; i++)
    {
        fifoCount = mpu.getFIFOCount();
        while (fifoCount < packetSize)
        {
            // Serial.print('.');
            delay(40);
            fifoCount = mpu.getFIFOCount();
        }

        // Serial.println();
        Serial.print(millis());
        Serial.print("\t");

        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // display real acceleration, adjusted to remove gravity
        mpu.dmpGetQuaternion(&q, fifoBuffer); // orientation
        mpu.dmpGetGyro(gyro, fifoBuffer);     // angular_velocity
        mpu.dmpGetAccel(&aa, fifoBuffer);     // linear_acceleration
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        Serial.print(q.w);
        Serial.print("\t");
        Serial.print(q.x);
        Serial.print("\t");
        Serial.print(q.y);
        Serial.print("\t");
        Serial.print(q.z);
        Serial.print("\t - \t");

        Serial.print(gyro[0]);
        Serial.print("\t");
        Serial.print(gyro[1]);
        Serial.print("\t");
        Serial.print(gyro[2]);
        Serial.print("\t - \t");

        Serial.print(aa.x);
        Serial.print("\t");
        Serial.print(aa.y);
        Serial.print("\t");
        Serial.println(aa.z);

        predelej to do bobik lib mpu9150.cpp
        pak udelej konverzi jednotek v bobik_bridge
        pak testuj v rviz
    }
}

int f2i(float f)
{
    return round(f * 1000);
}

void setup()
{
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    Serial.begin(115200);
    delay(1000);

    mpu.initialize();
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    DEBUG_PRINTLN(F("Setting DLPF bandwidth to 21Hz..."));
    mpu.setDLPFMode(MPU9150_DLPF_BW_20);            // 21Hz
    mpu.setFullScaleGyroRange(MPU9150_GYRO_FS_250); // [deg/sec]
    // accell is set to +/- 2g already
    // conversion will be done in bobik_bridge

    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

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
