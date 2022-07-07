#include "mpu9150.h"
#include "MPU9150_9Axis_MotionApps41.h"
#include <protocol.h>

// https://cdn.sparkfun.com/datasheets/Sensors/IMU/MPU-9150-Register-Map.pdf
// http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/Imu.html
// Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
// conversion from sensor units will be done in bobik_bridge

MPU9150 mpu;
uint16_t packetSize = 0;
uint8_t devStatus = 0;

mpu9150::mpu9150()
{
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    mpu.setDLPFMode(MPU9150_DLPF_BW_20);            // 21Hz
    mpu.setFullScaleGyroRange(MPU9150_GYRO_FS_250); // [deg/sec]
                                                    // accell is set to +/- 2g already
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

mpu9150::~mpu9150()
{
}

uint8_t mpu9150::readSensorQAG(MsgIMU9DOF_t *msg)
{
    if (packetSize == 0)
        return -1;

    uint16_t fifoCount = mpu.getFIFOCount();
    uint8_t fifoBuffer[64];
    Quaternion q;    // [w, x, y, z]         quaternion container
    VectorInt16 aa;  // [x, y, z]            accel sensor measurements
    int16_t gyro[3]; // [x, y, z]            gyro sensor measurements

    while (fifoCount >= packetSize)
    {
        mpu.dmpGetQuaternion(&q, fifoBuffer); // orientation
        mpu.dmpGetGyro(gyro, fifoBuffer);     // angular_velocity
        mpu.dmpGetAccel(&aa, fifoBuffer);     // linear_acceleration

        msg->qw = q.w;
        msg->qx = q.x;
        msg->qy = q.y;
        msg->qz = q.z;

        msg->gx = gyro[0];
        msg->gy = gyro[1];
        msg->gz = gyro[2];

        msg->ax = aa.x;
        msg->ay = aa.y;
        msg->az = aa.z;

        fifoCount -= packetSize;
    }

    return 0;
}
