#include "mpu9150.h"
#include "MPU9150_9Axis_MotionApps41.h"
#include <protocol.h>

// https://cdn.sparkfun.com/datasheets/Sensors/IMU/MPU-9150-Register-Map.pdf
// http://docs.ros.org/en/lunar/api/sensor_msgs/html/msg/Imu.html
// Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
// conversion from sensor units will be done in bobik_bridge

MPU9150 mpu;
uint16_t packetSize = 0; // normally 48
uint8_t devStatus = 0;

mpu9150::mpu9150()
{
}

void mpu9150::init()
{
    mpu.initialize();
    mpu.setDLPFMode(MPU9150_DLPF_BW_20);            // 21Hz
    mpu.setFullScaleGyroRange(MPU9150_GYRO_FS_250); // [deg/sec]
    mpu.setFullScaleAccelRange(MPU9150_ACCEL_FS_2); // +/- 2g
    devStatus = mpu.dmpInitialize();

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

uint8_t mpu9150::readSensorQAG(MsgIMU9DOF_t *msg)
{
    if (packetSize == 0)
        return -1;

    uint16_t fifoCount = mpu.getFIFOCount();
    uint8_t fifoBuffer[64];
    Quaternion q;    // [w, x, y, z]         quaternion container
    VectorInt16 aa;  // [x, y, z]            accel sensor measurements
    VectorInt16 aa_sum;
    int16_t gyro[3]; // [x, y, z]            gyro sensor measurements
    int16_t gyro_sum[3];
    int16_t mag[3];  // [x, y, z]            magnetometer, from -4096 to +4095 in decimal -> Magnetic flux density [µT] -1229 to 1229

    float count = 0;
    gyro_sum[0] = 0;
    gyro_sum[1] = 0;
    gyro_sum[2] = 0;
    while (fifoCount >= packetSize)
    {
        count++;
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.dmpGetQuaternion(&q, fifoBuffer); // orientation
        mpu.dmpGetMag(mag, fifoBuffer);       // raw orientation
        mpu.dmpGetGyro(gyro, fifoBuffer);     // angular_velocity
        mpu.dmpGetAccel(&aa, fifoBuffer);     // linear_acceleration

        // Always take last value
        msg->qw = (int16_t)(q.w * FLOAT_INT16_PRECISION);
        msg->qx = (int16_t)(q.x * FLOAT_INT16_PRECISION);
        msg->qy = (int16_t)(q.y * FLOAT_INT16_PRECISION);
        msg->qz = (int16_t)(q.z * FLOAT_INT16_PRECISION);

        // Average
        gyro_sum[0] += gyro[0];
        gyro_sum[1] += gyro[1];
        gyro_sum[2] += gyro[2];

        // Average
        aa_sum.x += aa.x;
        aa_sum.y += aa.y;
        aa_sum.z += aa.z;

        fifoCount -= packetSize;
    }

    msg->mx = mag[0];
    msg->my = mag[1];
    msg->mz = mag[2];

    msg->gx = (int16_t)((float)gyro_sum[0] / count);
    msg->gy = (int16_t)((float)gyro_sum[1] / count);
    msg->gz = (int16_t)((float)gyro_sum[2] / count);

    msg->ax = (int16_t)((float)aa_sum.x / count);
    msg->ay = (int16_t)((float)aa_sum.y / count);
    msg->az = (int16_t)((float)aa_sum.z / count);

    return 0;
}
