#!/usr/bin/python3
import time, sys
import numpy as np

import teensyimu as ti

At = []
Ax = []
Ay = []
Az = []
Gx = []
Gy = []
Gz = []
U = []
last_t_us = 0

driver = None

def imu_cb(msg):
    global last_t_us
    dt = (msg.t_us - last_t_us) * 1e-6 # us to s
    last_t_us = msg.t_us
    hz = 1./dt
    print('Got IMU at {} us ({:.0f} Hz): {:.2f}, {:.2f}, {:.2f}, \t {:.2f}, {:.2f}, {:.2f}'
            .format(msg.t_us, hz,
                    msg.accel_x, msg.accel_y, msg.accel_z,
                    msg.gyro_x, msg.gyro_y, msg.gyro_z))

    At.append(msg.t_us * 1e-6)
    Ax.append(msg.accel_x)
    Ay.append(msg.accel_y)
    Az.append(msg.accel_z)
    Gx.append(msg.gyro_x)
    Gy.append(msg.gyro_y)
    Gz.append(msg.gyro_z)

    # u = 0.5
    t = msg.t_us * 1e-6
    u = 0.6 + 0.4 * np.cos(2 * np.pi * 0.5 * t * t)
    U.append(u)
    uu = int(u * 1000)
    driver.sendMotorCmd(uu)

def main():
    port = ti.tools.find_teensy_or_die()

    global driver
    driver = ti.SerialDriver(port)
    time.sleep(0.1)

    driver.sendMotorCmd(500)
    time.sleep(1)

    driver.sendRate(1000)
    time.sleep(0.001)

    driver.registerCallbackIMU(imu_cb)

    try:
        time.sleep(10)
    except KeyboardInterrupt:
        pass

    # clean up to prevent error or resource deadlock
    driver.unregisterCallbacks()

    # shut off motor
    driver.sendMotorCmd(0)

    data = np.array([At, Ax, Ay, Az, Gx, Gy, Gz, U]).T
    np.savetxt("datasysid.csv", data, delimiter=',')

if __name__ == '__main__':
    main()