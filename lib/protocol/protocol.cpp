#include "protocol.h"
#include <Arduino.h>
#include "CRC8.h"
#include "ievent_handler.h"

#define SERIAL_TMOUT 5

CRC8 crc8 = CRC8();
uint8_t recv_buf[64];

IEventHandler *_handler_obj;

void protocol_init() {
    Serial3.begin(19200);  //500000 on USB, 115200 on Serial3 confimed on oscilloscope
    Serial3.setTimeout(SERIAL_TMOUT);
}

void emit_loop_start() {
    crc8.restart();

    Serial3.write(LOOP_START);
    crc8.add(LOOP_START);
    Serial3.write(LOOP_START);
    crc8.add(LOOP_START);
    Serial3.write(LOOP_START);
    crc8.add(LOOP_START);

    emit1(LOOP_TIMESTAMP, millis());
}

void emit_loop_end() {
    uint8_t crc = crc8.getCRC();
    emit1(LOOP_END_MSG, (long)crc); //udelat zpravy uint_8t

    Serial3.write(LOOP_END);
    crc8.add(LOOP_END);
    Serial3.write(LOOP_END);
    crc8.add(LOOP_END);
    Serial3.write(LOOP_END);
    crc8.add(LOOP_END);
}

void emit1(unsigned char type, long value) {
    Serial3.write(MSG_START);
    crc8.add(MSG_START);

    Serial3.write(type);
    crc8.add(type);

    byte buf[4];
    buf[0] = value & 255;
    buf[1] = (value >> 8)  & 255;
    buf[2] = (value >> 16) & 255;
    buf[3] = (value >> 24) & 255;
    Serial3.write(buf, sizeof(buf));
    for (int i=0; i<4; i++) crc8.add(buf[i]);
}

void emit1(unsigned char type, unsigned long value) {
    Serial3.write(MSG_START);
    crc8.add(MSG_START);

    Serial3.write(type);
    crc8.add(type);

    byte buf[4];
    buf[0] = value & 255;
    buf[1] = (value >> 8)  & 255;
    buf[2] = (value >> 16) & 255;
    buf[3] = (value >> 24) & 255;
    Serial3.write(buf, sizeof(buf));
    for (int i=0; i<4; i++) crc8.add(buf[i]);
}

void emit4(unsigned char type, uint8_t v1, uint8_t v2, uint8_t v3, uint8_t v4) {
    Serial3.write(MSG_START);
    crc8.add(MSG_START);

    Serial3.write(type);
    crc8.add(type);

    byte buf[4] = {v1,v2,v3,v4};
    Serial3.write(buf, sizeof(buf));
    for (int i=0; i<4; i++) crc8.add(buf[i]);
}

void emit_caster_joint_states(MsgCasterJointStates_t *cjs) {
    Serial3.write(MSG_START);
    crc8.add(MSG_START);

    Serial3.write(CASTER_JOINT_STATES);
    crc8.add(CASTER_JOINT_STATES);

    cjs->fl_caster_drive_joint_redundant = cjs->fl_caster_drive_joint;
    cjs->fr_caster_drive_joint_redundant = cjs->fr_caster_drive_joint;
    cjs->r_caster_drive_joint_redundant = cjs->r_caster_drive_joint;
    uint8_t *buf = (uint8_t*)cjs;
    Serial3.write(buf, sizeof(MsgCasterJointStates_t));
    for (int i=0; i<(6*2+2); i++) crc8.add(buf[i]);
}

void emit_IMU9DOF(MsgIMU9DOF_t *imu) {
    Serial3.write(MSG_START);
    crc8.add(MSG_START);

    Serial3.write(IMU_9DOF);
    crc8.add(IMU_9DOF);

    uint8_t *buf = (uint8_t*)imu;
    Serial3.write(buf, sizeof(MsgIMU9DOF_t));
    for (int i=0; i<18; i++) crc8.add(buf[i]);
}

// For testing purposes
/*
void dispatch_void(unsigned long timeout, unsigned long granularity, uint8_t *log_buf) {
  delay(50);
  for (int i=0; i<100; i++) {
    if (Serial3.available()) {
      Serial3.read();
    }
    else {
      return;
    }
  }
}
*/

void dispatch_until(unsigned long timeout, unsigned long granularity, uint8_t *log_buf) {
  while(1) {  // Read and wait cycle
    if (Serial3.available() >= 4) {
      if (Serial3.peek() == MSG_2BYTES) {
        Serial3.readBytes(recv_buf, 4);
        dispatch_tohandlers(recv_buf[1], log_buf);
        // memcpy(log_buf, recv_buf, 4);
      }
      else if (Serial3.peek() == MSG_6BYTES) {
        if (Serial3.available() >= 8) {
          Serial3.readBytes(recv_buf, 8);
          dispatch_tohandlers(recv_buf[1], log_buf);
          // memcpy(log_buf, recv_buf, 8);
        }
        else continue;
      }
      else if (Serial3.peek() == MSG_12BYTES) {
        if (Serial3.available() >= 14) {
          Serial3.readBytes(recv_buf, 14);
          dispatch_tohandlers(recv_buf[1], log_buf);
          // memcpy(log_buf, recv_buf+2, 4);
        }
        else continue;
      }
      else { //unknown data, read one byte and throuw away
        Serial3.read();  
        continue;
      }
    }

    // Wait for more data to come or return to keep required loop frequency
    unsigned long current_ms = millis();
    if ( (current_ms + granularity) < timeout ) {
      if (Serial3.available() < 4) {
        delay(granularity);  // delay only if receive buffer is empty
      }
    }
    else {
      return;
    }
  }
}

void dispatch_tohandlers(uint8_t msg_type, uint8_t *log_buf) {
  // TODO _handler_obj = map.get(msg_type);
  _handler_obj->serial_message_handler(recv_buf+2, log_buf);  //skip message header and type
}

void serial_event_message_subscribe(uint8_t msg_type, IEventHandler *handler_obj) {
  _handler_obj = handler_obj;
}
