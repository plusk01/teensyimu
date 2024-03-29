/**
 * @file screenprint.cpp
 * @brief Example application that dumps teensy IMU data to terminal
 * @author Parker Lusk <plusk@mit.edu>
 * @date 21 Nov 2020
 */

#include <chrono>
#include <future>
#include <iomanip>
#include <iostream>
#include <thread>
#include <sstream>

#include <teensyimu/serial_driver.h>

/// |brief Global variables
uint32_t last_t_us = 0;

/**
 * @brief      Handles IMU messages when received via serial
 *
 * @param[in]  msg   The unpacked IMU message
 */
void callback(const ti_serial_imu_msg_t& msg)
{
  const double dt = (msg.t_us - last_t_us) * 1e-6; // us to s
  const double hz = 1. / dt;
  last_t_us = msg.t_us;

  static constexpr int w = 5;
  std::stringstream ss;
  ss << std::fixed << std::setprecision(2)
     << std::setw(w) << std::setfill(' ')
     << msg.accel_x << ", "
     << std::setw(w) << std::setfill(' ')
     << msg.accel_y << ", "
     << std::setw(w) << std::setfill(' ')
     << msg.accel_z
     << '\t'
     << std::setw(w) << std::setfill(' ')
     << msg.gyro_x << ", "
     << std::setw(w) << std::setfill(' ')
     << msg.gyro_y << ", "
     << std::setw(w) << std::setfill(' ')
     << msg.gyro_z
     << '\t'
     << std::setw(w) << std::setfill(' ')
     << msg.mag_x << ", "
     << std::setw(w) << std::setfill(' ')
     << msg.mag_y << ", "
     << std::setw(w) << std::setfill(' ')
     << msg.mag_z;

  std::cout << "Got IMU at " << msg.t_us << " us (" << hz << " Hz): "
            << ss.str() << std::endl;
}

void rateCb(const ti_serial_rate_msg_t& msg)
{
  std::cout << std::endl << "***************************" << std::endl;
  std::cout << "Sample Rate: " << msg.frequency << " Hz" << std::endl;
  std::cout << "***************************" << std::endl << std::endl;
}

int main(int argc, char const *argv[])
{
  std::string port = "/dev/ttyACM0";

  if (argc == 2) port = std::string(argv[1]);

  acl::teensyimu::SerialDriver driver(port);
  driver.registerCallbackIMU(callback);
  driver.registerCallbackRate(rateCb);

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  driver.sendRate(100);

  // spin forever and let CPU do other things (no busy waiting)
  std::promise<void>().get_future().wait();
  return 0;
}
