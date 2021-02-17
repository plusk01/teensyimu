#!/usr/bin/python3
import time, sys
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui

from acl_serial_driver import ACLSerialDriver, ACLSerialRateMsg

At = []
Ax = []
Ay = []
Az = []
last_t_us = 0

# how many seconds of samples to keep in the plotting window?
SAMPLE_WINDOW_SEC = 5

def imu_cb(msg):
    global last_t_us
    dt = (msg.t_us - last_t_us) * 1e-6 # us to s
    last_t_us = msg.t_us
    hz = 1./dt
    print('Got IMU at {} us ({:.0f} Hz): {:.2f}, {:.2f}, {:.2f}, \t {:.2f}, {:.2f}, {:.2f}'
            .format(msg.t_us, hz,
                    msg.accel_x, msg.accel_y, msg.accel_z,
                    msg.gyro_x, msg.gyro_y, msg.gyro_z))

    At.append(msg.t_us)
    Ax.append(msg.accel_x)
    Ay.append(msg.accel_y)
    Az.append(msg.accel_z)

    if len(At) > hz*SAMPLE_WINDOW_SEC:
      At.pop(0)
      Ax.pop(0)
      Ay.pop(0)
      Az.pop(0)

def find_teensy():
    import serial.tools.list_ports
    for port in serial.tools.list_ports.comports():
        if port.manufacturer and port.manufacturer.lower() == 'teensyduino':
            return port.device
    return None

def main():
    port = find_teensy()
    if port is None:
        print("Could not find Teensy!")
        sys.exit()

    driver = ACLSerialDriver(port)
    time.sleep(0.1)
    driver.registerCallbackIMU(imu_cb)
    driver.sendRate(ACLSerialRateMsg(500))

    # https://pyqtgraph.readthedocs.io/en/latest/plotting.html#examples
    pw = pg.plot(title="Accelerometer")
    timer = pg.QtCore.QTimer()
    def update():
        pw.plot(At, Ax, pen=(1,3), clear=True)
        pw.plot(At, Ay, pen=(2,3))
        pw.plot(At, Az, pen=(3,3))
        QtGui.QApplication.processEvents()

    timer.timeout.connect(update)
    timer.start(50) # ms
    QtGui.QApplication.instance().exec_()

    # clean up to prevent error or resource deadlock
    driver.unregisterCallbacks()

if __name__ == '__main__':
    main()