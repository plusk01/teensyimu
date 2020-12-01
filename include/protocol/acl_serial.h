/**
 * @file acl_serial.h
 * @brief Serial protocol for ACL teensy-imu 
 * @author Parker Lusk <plusk@mit.edu>
 * @date 21 Nov 2020
 * 
 * cf., https://magiccvs.byu.edu/gitlab/hummingbird
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

//=============================================================================
// message types
//=============================================================================

typedef enum {
  ACL_SERIAL_MSG_IMU,
  ACL_SERIAL_NUM_MSGS
} acl_msg_type_t;

//=============================================================================
// message definitions
//=============================================================================

typedef struct {
  uint32_t t_us;
  float accel_x;
  float accel_y;
  float accel_z;
  float gyro_x;
  float gyro_y;
  float gyro_z;
} acl_serial_imu_msg_t;

// payload lengths
static constexpr float ACL_SERIAL_PAYLOAD_LEN[] = {
  sizeof(acl_serial_imu_msg_t)
};
static constexpr size_t ACL_SERIAL_MAX_PAYLOAD_LEN = sizeof(acl_serial_imu_msg_t);

//=============================================================================
// generic message type
//=============================================================================

static constexpr uint8_t ACL_SERIAL_MAGIC = 0xA5;

struct __attribute__((packed)) acl_serial_message_t {
  uint8_t magic;
  uint8_t type;
  uint8_t payload[ACL_SERIAL_MAX_PAYLOAD_LEN];
  uint8_t crc;
};

static constexpr size_t ACL_SERIAL_MAX_MESSAGE_LEN = sizeof(acl_serial_message_t);

//=============================================================================
// utility functions
//=============================================================================

// source: http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html#gab27eaaef6d7fd096bd7d57bf3f9ba083
inline uint8_t acl_serial_update_crc(uint8_t inCrc, uint8_t inData)
{
  uint8_t i;
  uint8_t data;

  data = inCrc ^ inData;

  for ( i = 0; i < 8; i++ )
  {
    if (( data & 0x80 ) != 0 )
    {
      data <<= 1;
      data ^= 0x07;
    }
    else
    {
      data <<= 1;
    }
  }
  return data;
}

inline void acl_serial_finalize_message(acl_serial_message_t *msg)
{
  msg->magic = ACL_SERIAL_MAGIC;

  uint8_t crc = 0;
  crc = acl_serial_update_crc(crc, msg->magic);
  crc = acl_serial_update_crc(crc, msg->type);
  for (size_t i=0; i<ACL_SERIAL_PAYLOAD_LEN[msg->type]; ++i)
  {
    crc = acl_serial_update_crc(crc, msg->payload[i]);
  }

  msg->crc = crc;
}

inline size_t acl_serial_send_to_buffer(uint8_t *dst, const acl_serial_message_t *src)
{
  size_t offset = 0;
  memcpy(dst + offset, &src->magic,  sizeof(src->magic)); offset += sizeof(src->magic);
  memcpy(dst + offset, &src->type,   sizeof(src->type));  offset += sizeof(src->type);
  memcpy(dst + offset, src->payload, ACL_SERIAL_PAYLOAD_LEN[src->type]); offset += ACL_SERIAL_PAYLOAD_LEN[src->type];
  memcpy(dst + offset, &src->crc,    sizeof(src->crc)); offset += sizeof(src->crc);
  return offset;
}

//=============================================================================
// IMU message
//=============================================================================

inline void acl_serial_imu_msg_pack(acl_serial_message_t *dst, const acl_serial_imu_msg_t *src)
{
  dst->type = ACL_SERIAL_MSG_IMU;
  size_t offset = 0;
  memcpy(dst->payload + offset, &src->t_us, sizeof(src->t_us)); offset += sizeof(src->t_us);
  memcpy(dst->payload + offset, &src->accel_x, sizeof(src->accel_x)); offset += sizeof(src->accel_x);
  memcpy(dst->payload + offset, &src->accel_y, sizeof(src->accel_y)); offset += sizeof(src->accel_y);
  memcpy(dst->payload + offset, &src->accel_z, sizeof(src->accel_z)); offset += sizeof(src->accel_z);
  memcpy(dst->payload + offset, &src->gyro_x,  sizeof(src->gyro_x));  offset += sizeof(src->gyro_x);
  memcpy(dst->payload + offset, &src->gyro_y,  sizeof(src->gyro_y));  offset += sizeof(src->gyro_y);
  memcpy(dst->payload + offset, &src->gyro_z,  sizeof(src->gyro_z));
  acl_serial_finalize_message(dst);
}

inline void acl_serial_imu_msg_unpack(acl_serial_imu_msg_t *dst, const acl_serial_message_t *src)
{
  size_t offset = 0;
  memcpy(&dst->t_us, src->payload + offset, sizeof(dst->t_us)); offset += sizeof(dst->t_us);
  memcpy(&dst->accel_x, src->payload + offset, sizeof(dst->accel_x)); offset += sizeof(dst->accel_x);
  memcpy(&dst->accel_y, src->payload + offset, sizeof(dst->accel_y)); offset += sizeof(dst->accel_y);
  memcpy(&dst->accel_z, src->payload + offset, sizeof(dst->accel_z)); offset += sizeof(dst->accel_z);
  memcpy(&dst->gyro_x,  src->payload + offset, sizeof(dst->gyro_x));  offset += sizeof(dst->gyro_x);
  memcpy(&dst->gyro_y,  src->payload + offset, sizeof(dst->gyro_y));  offset += sizeof(dst->gyro_y);
  memcpy(&dst->gyro_z,  src->payload + offset, sizeof(dst->gyro_z));
}

inline size_t acl_serial_imu_msg_send_to_buffer(uint8_t *dst, const acl_serial_imu_msg_t *src)
{
  acl_serial_message_t msg;
  acl_serial_imu_msg_pack(&msg, src);
  return acl_serial_send_to_buffer(dst, &msg);
}

//==============================================================================
// parser
//==============================================================================

typedef enum
{
  ACL_SERIAL_PARSE_STATE_IDLE,
  ACL_SERIAL_PARSE_STATE_GOT_MAGIC,
  ACL_SERIAL_PARSE_STATE_GOT_TYPE,
  ACL_SERIAL_PARSE_STATE_GOT_PAYLOAD
} acl_serial_parse_state_t;

inline bool acl_serial_parse_byte(uint8_t byte, acl_serial_message_t *msg)
{
  static acl_serial_parse_state_t parse_state = ACL_SERIAL_PARSE_STATE_IDLE;
  static uint8_t crc_value = 0;
  static size_t payload_index = 0;
  static acl_serial_message_t msg_buffer;

  bool got_message = false;
  switch (parse_state)
  {
  case ACL_SERIAL_PARSE_STATE_IDLE:
    if (byte == ACL_SERIAL_MAGIC)
    {
      crc_value = 0;
      payload_index = 0;

      msg_buffer.magic = byte;
      crc_value = acl_serial_update_crc(crc_value, byte);

      parse_state = ACL_SERIAL_PARSE_STATE_GOT_MAGIC;
    }
    break;

  case ACL_SERIAL_PARSE_STATE_GOT_MAGIC:
    msg_buffer.type = byte;
    crc_value = acl_serial_update_crc(crc_value, byte);
    parse_state = ACL_SERIAL_PARSE_STATE_GOT_TYPE;
    break;

  case ACL_SERIAL_PARSE_STATE_GOT_TYPE:
    msg_buffer.payload[payload_index++] = byte;
    crc_value = acl_serial_update_crc(crc_value, byte);
    if (payload_index == ACL_SERIAL_PAYLOAD_LEN[msg_buffer.type])
    {
      parse_state = ACL_SERIAL_PARSE_STATE_GOT_PAYLOAD;
    }
    break;

  case ACL_SERIAL_PARSE_STATE_GOT_PAYLOAD:
    msg_buffer.crc = byte;
    if (msg_buffer.crc == crc_value)
    {
      got_message = true;
      memcpy(msg, &msg_buffer, sizeof(msg_buffer));
    }
    parse_state = ACL_SERIAL_PARSE_STATE_IDLE;
    break;
  }

  return got_message;
}
