/**
 * @file serial_driver.h
 * @brief Serial communication driver for teensy-imu
 * @author Parker Lusk <plusk@mit.edu>
 * @date 21 Nov 2020
 */

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "protocol/acl_serial.h"

namespace async_comm { class Serial; }

namespace acl {
namespace teensyimu {

  using CallbackIMU = std::function<void(const acl_serial_imu_msg_t&)>;
  using CallbackRate = std::function<void(const acl_serial_rate_msg_t&)>;

  class SerialDriver
  {
  public:
    SerialDriver(const std::string& port = "/dev/ttyACM0", uint32_t baud = 115200);
    ~SerialDriver();

    void sendRate(const acl_serial_rate_msg_t& msg);
    void sendMotorCmd(const acl_serial_motorcmd_msg_t& msg);

    void registerCallbackIMU(CallbackIMU cb);
    void registerCallbackRate(CallbackRate cb);
    void unregisterCallbacks();
    
  private:
    std::unique_ptr<async_comm::Serial> serial_;
    CallbackIMU cb_imu_;
    CallbackRate cb_rate_;
    std::mutex mtx_; ///< synchronize callback resource reg/unreg

    void callback(const uint8_t * data, size_t len);

    void handleIMUMsg(const acl_serial_message_t& msg);
    void handleRateMsg(const acl_serial_message_t& msg);
  };

} // ns teensyimu
} // ns acl
