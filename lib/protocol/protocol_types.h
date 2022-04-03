#ifndef protocol_types
#define protocol_types

#define FLOAT_INT16_PRECISION 5000.0

#define LOOP_START 0xEA
#define LOOP_END 0xAE
#define MSG_START 0xEE

//Message types Arduino -> Driver
#define LOG4 1
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

#define CASTER_JOINT_STATES 2
struct MsgCasterJointStates_t {
   int16_t fl_caster_rotation_joint;
   int16_t fr_caster_rotation_joint;
   int16_t r_caster_rotation_joint;
};

#define DRIVE_JOINT_STATES 3
struct MsgDriveJointStates_t {
   int16_t fl_caster_drive_joint;
   int16_t fr_caster_drive_joint;
   int16_t r_caster_drive_joint;
};

#define IMU_9DOF 4
struct MsgIMU9DOF_t {
   int16_t ax, ay, az;
   int16_t gx, gy, gz;
   int16_t mx, my, mz;
};

#define LOADCELL_UPPER_ARM_LIFT_JOINT 5
struct MsgLoadCell_t {
   int32_t value;
};



//Mesage types Driver -> Arduino
#define MSG_2BYTES 0xFA
#define MSG_6BYTES 0xFB
#define MSG_12BYTES 0xFC

#define DRIVE_CMD 0x01
struct MsgCmdVel_t {
   int16_t linear_x;
   int16_t linear_y;
   int16_t rotation;
};

#endif