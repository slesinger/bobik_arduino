#ifndef protocol
#define protocol

#include <stdint.h>
#include "protocol_types.h"
#include "ievent_handler.h"

void protocol_init();
void emit_loop_start();
void emit_loop_end();

void emit1(unsigned char type, long value);
void emit1(unsigned char type, unsigned long value);
void emit4(unsigned char type, uint8_t v1, uint8_t v2, uint8_t v3, uint8_t v4);
void emit_caster_joint_states(MsgCasterJointStates_t *cjs);
void emit_drive_joint_states(MsgDriveJointStates_t *cjs);
void emit_IMU9DOF(MsgIMU9DOF_t *imu);

void dispatch_void(unsigned long timeout, unsigned long granularity, uint8_t *log_buf);
void dispatch_until(unsigned long timeout, unsigned long granularity, uint8_t *log_buf);
void dispatch_tohandlers(uint8_t msg_type, uint8_t *log_buf);
void serial_event_message_subscribe(uint8_t msg_type, IEventHandler *handler_obj);

#endif