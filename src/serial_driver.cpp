/**
 * @file serial_driver.cpp
 * @brief Serial communication driver for teensy-imu
 * @author Parker Lusk <plusk@mit.edu>
 * @date 21 Nov 2020
 */

#include <iostream>
#include <stdexcept>

#include <async_comm/serial.h>

#include "teensyimu/serial_driver.h"

namespace acl {
namespace teensyimu {

SerialDriver::SerialDriver(const std::string& port, uint32_t baud)
{
  using namespace std::placeholders;

  serial_.reset(new async_comm::Serial(port, baud));
  serial_->register_receive_callback(
            std::bind(&SerialDriver::callback, this, _1, _2));

  if (!serial_->init()) {
    throw std::runtime_error("Could not open serial port '" + port + "'");
  }
}

// ----------------------------------------------------------------------------

SerialDriver::~SerialDriver()
{
  serial_->close();
}

// ----------------------------------------------------------------------------
// Send message methods
// ----------------------------------------------------------------------------

void SerialDriver::sendRate(uint16_t frequency)
{
  // put into standard message
  ti_serial_rate_msg_t msg;
  msg.frequency = frequency;

  uint8_t buf[TI_SERIAL_MAX_MESSAGE_LEN];
  const size_t len = ti_serial_rate_msg_send_to_buffer(buf, &msg);
  serial_->send_bytes(buf, len);
}

// ----------------------------------------------------------------------------

void SerialDriver::sendMotorCmd(double percentage)
{
  // put into standard message
  ti_serial_motorcmd_msg_t msg;
  msg.percentage = static_cast<uint16_t>(percentage * 10);

  uint8_t buf[TI_SERIAL_MAX_MESSAGE_LEN];
  const size_t len = ti_serial_motorcmd_msg_send_to_buffer(buf, &msg);
  serial_->send_bytes(buf, len);
}

// ----------------------------------------------------------------------------
// Callback stuff
// ----------------------------------------------------------------------------

void SerialDriver::registerCallbackIMU(CallbackIMU cb)
{
  std::lock_guard<std::mutex> lock(mtx_);
  cb_imu_ = cb;
}

// ----------------------------------------------------------------------------

void SerialDriver::registerCallbackIMU_NoMag(CallbackIMU_NoMag cb)
{
  std::lock_guard<std::mutex> lock(mtx_);
  cb_imu_nomag_ = cb;
}

// ----------------------------------------------------------------------------

void SerialDriver::registerCallbackIMU_3DOF(CallbackIMU_3DOF cb)
{
  std::lock_guard<std::mutex> lock(mtx_);
  cb_imu_3dof_ = cb;
}

// ----------------------------------------------------------------------------

void SerialDriver::registerCallbackRate(CallbackRate cb)
{
  std::lock_guard<std::mutex> lock(mtx_);
  cb_rate_ = cb;
}

// ----------------------------------------------------------------------------

void SerialDriver::unregisterCallbacks()
{
  std::lock_guard<std::mutex> lock(mtx_);
  cb_imu_ = nullptr;
  cb_imu_nomag_ = nullptr;
  cb_imu_3dof_ = nullptr;
  cb_rate_ = nullptr;
}


// ----------------------------------------------------------------------------
// Private Methods
// ----------------------------------------------------------------------------

void SerialDriver::callback(const uint8_t * data, size_t len)
{
  ti_serial_message_t msg;
  for (size_t i=0; i<len; ++i) {
    if (ti_serial_parse_byte(data[i], &msg)) {

      switch (msg.type) {
        case TI_SERIAL_MSG_IMU:
          handleIMUMsg(msg);
          break;
        case TI_SERIAL_MSG_IMU_NOMAG:
          handleIMUNoMagMsg(msg);
          break;
        case TI_SERIAL_MSG_IMU_3DOF:
          handleIMU3DOFMsg(msg);
          break;
        case TI_SERIAL_MSG_RATE:
          handleRateMsg(msg);
          break;
      }

    }
  }
}

// ----------------------------------------------------------------------------

void SerialDriver::handleIMUMsg(const ti_serial_message_t& msg)
{
  ti_serial_imu_msg_t imu;
  ti_serial_imu_msg_unpack(&imu, &msg);

  std::lock_guard<std::mutex> lock(mtx_);
  if (cb_imu_) cb_imu_(imu);
}

// ----------------------------------------------------------------------------

void SerialDriver::handleIMUNoMagMsg(const ti_serial_message_t& msg)
{
  ti_serial_imu_nomag_msg_t imu;
  ti_serial_imu_nomag_msg_unpack(&imu, &msg);

  std::lock_guard<std::mutex> lock(mtx_);
  if (cb_imu_nomag_) cb_imu_nomag_(imu);
}

// ----------------------------------------------------------------------------

void SerialDriver::handleIMU3DOFMsg(const ti_serial_message_t& msg)
{
  ti_serial_imu_3dof_msg_t imu;
  ti_serial_imu_3dof_msg_unpack(&imu, &msg);

  std::lock_guard<std::mutex> lock(mtx_);
  if (cb_imu_3dof_) cb_imu_3dof_(imu);
}

// ----------------------------------------------------------------------------

void SerialDriver::handleRateMsg(const ti_serial_message_t& msg)
{
  ti_serial_rate_msg_t rate;
  ti_serial_rate_msg_unpack(&rate, &msg);

  std::lock_guard<std::mutex> lock(mtx_);
  if (cb_rate_) cb_rate_(rate);
}

} // ns teensyimu
} // ns acl
