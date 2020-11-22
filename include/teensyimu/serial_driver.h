/**
 * @file serial_driver.h
 * @brief Serial communication driver for teensy-imu
 * @author Parker Lusk <plusk@mit.edu>
 * @date 21 Nov 2020
 */

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "protocol/acl_serial.h"

namespace async_comm { class Serial; }

namespace acl {
namespace teensyimu {

  using CallbackIMU = std::function<void(const acl_serial_imu_msg_t&)>;

  class SerialDriver
  {
  public:
    SerialDriver(std::string port = "/dev/ttyACM0", uint32_t baud = 115200);
    ~SerialDriver();

    void registerCallbackIMU(CallbackIMU cb);
    
  private:
    std::unique_ptr<async_comm::Serial> serial_;
    CallbackIMU cb_imu_;

    void callback(const uint8_t * data, size_t len);

    void handleIMUMsg(const acl_serial_message_t& msg);
  };

} // ns teensyimu
} // ns acl
