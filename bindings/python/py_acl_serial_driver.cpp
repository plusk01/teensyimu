/**
 * @file py_acl_serial_driver.cpp
 * @brief Python bindings for acl_serial_driver
 * @author Parker Lusk <plusk@mit.edu>
 * @date 22 Nov 2020
 */

#include <cstdint>

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>

#include <protocol/acl_serial.h>
#include <teensyimu/serial_driver.h>

namespace py = pybind11;

PYBIND11_MODULE(acl_serial_driver, m)
{
  m.doc() = "ACL serial driver for talking to Teensy-IMU";
  m.attr("__version__") = "0.1";

  py::class_<acl_serial_imu_msg_t>(m, "ACLSerialIMUMsg")
    .def_readwrite("t_ms", &acl_serial_imu_msg_t::t_ms)
    .def_readwrite("accel_x", &acl_serial_imu_msg_t::accel_x)
    .def_readwrite("accel_y", &acl_serial_imu_msg_t::accel_y)
    .def_readwrite("accel_z", &acl_serial_imu_msg_t::accel_z)
    .def_readwrite("gyro_x", &acl_serial_imu_msg_t::gyro_x)
    .def_readwrite("gyro_y", &acl_serial_imu_msg_t::gyro_y)
    .def_readwrite("gyro_z", &acl_serial_imu_msg_t::gyro_z);

  py::class_<acl::teensyimu::SerialDriver>(m, "ACLSerialDriver")
    .def(py::init<const std::string&, uint32_t>(),
          py::arg("port")="/dev/ttyACM0", py::arg("baud")=115200)
    .def("registerCallbackIMU", &acl::teensyimu::SerialDriver::registerCallbackIMU);
}