#ifndef protocol_types
#define protocol_types

#define LOOP_START 0xEA
#define LOOP_END 0xAE
#define MSG_START 0xEE

//Message types Arduino -> Driver
#define LOG4 4
struct MsgLog4_t {
   uint8_t log1;
   uint8_t log2;
   uint8_t log3;
   uint8_t log4;
};

#define LOOP_END_MSG 0xFA
struct MsgCRC_t {
   uint8_t crc;
};

#define LOOP_TIMESTAMP 0xAA
struct MsgTimestamp_t {
   uint32_t timestamp;
};

#define LOADCELL_UPPER_ARM_LIFT_JOINT 1
struct MsgLoadCell_t {
   int32_t value;
};

#define CASTER_JOINT_STATES 2
struct MsgCasterJointStates_t {
   int16_t fl_caster_rotation_joint;
   int16_t fr_caster_rotation_joint;
   int16_t r_caster_rotation_joint;
};

#define IMU_9DOF 3
struct MsgIMU9DOF_t {
   int16_t ax, ay, az;
   int16_t gx, gy, gz;
   int16_t mx, my, mz;
};


//Mesage types Driver -> Arduino
#define MSG_2BYTES 0xFA
#define MSG_6BYTES 0xFB

#define DRIVE_CMD 0x01

#endif