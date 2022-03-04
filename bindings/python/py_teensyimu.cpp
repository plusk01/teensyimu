/**
 * @file py_teensyimu.cpp
 * @brief Python bindings for teensyimu
 * @author Parker Lusk <plusk@mit.edu>
 * @date 22 Nov 2020
 */

#include <cstdint>

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>

#include <protocol/teensyimu_serial.h>
#include <teensyimu/serial_driver.h>

namespace py = pybind11;

PYBIND11_MODULE(teensyimu, m)
{
  m.doc() = "Serial driver and tools for Teensy IMU board";
  m.attr("__version__") = PROJECT_VERSION;

  py::class_<ti_serial_imu_msg_t>(m, "SerialIMUMsg")
    .def_readwrite("t_us", &ti_serial_imu_msg_t::t_us)
    .def_readwrite("accel_x", &ti_serial_imu_msg_t::accel_x)
    .def_readwrite("accel_y", &ti_serial_imu_msg_t::accel_y)
    .def_readwrite("accel_z", &ti_serial_imu_msg_t::accel_z)
    .def_readwrite("gyro_x", &ti_serial_imu_msg_t::gyro_x)
    .def_readwrite("gyro_y", &ti_serial_imu_msg_t::gyro_y)
    .def_readwrite("gyro_z", &ti_serial_imu_msg_t::gyro_z)
    .def_readwrite("mag_x", &ti_serial_imu_msg_t::mag_x)
    .def_readwrite("mag_y", &ti_serial_imu_msg_t::mag_y)
    .def_readwrite("mag_z", &ti_serial_imu_msg_t::mag_z);

  py::class_<ti_serial_rate_msg_t>(m, "SerialRateMsg")
    .def(py::init<uint16_t>(),
          py::arg("frequency")=500)
    .def_readwrite("frequency", &ti_serial_rate_msg_t::frequency);

  py::class_<ti_serial_motorcmd_msg_t>(m, "SerialMotorCmdMsg")
    .def(py::init<uint16_t>(),
          py::arg("percentage")=0)
    .def_readwrite("percentage", &ti_serial_motorcmd_msg_t::percentage);

  py::class_<acl::teensyimu::SerialDriver>(m, "SerialDriver")
    .def(py::init<const std::string&, uint32_t>(),
          py::arg("port")="/dev/ttyACM0", py::arg("baud")=115200)
    .def("sendRate", &acl::teensyimu::SerialDriver::sendRate)
    .def("sendMotorCmd", &acl::teensyimu::SerialDriver::sendMotorCmd)
    .def("registerCallbackIMU", &acl::teensyimu::SerialDriver::registerCallbackIMU)
    .def("registerCallbackRate", &acl::teensyimu::SerialDriver::registerCallbackRate)
    .def("unregisterCallbacks", &acl::teensyimu::SerialDriver::unregisterCallbacks, py::call_guard<py::gil_scoped_release>());
}