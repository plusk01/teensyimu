import time, sys
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui

import matplotlib.pyplot as plt

from acl_serial_driver import ACLSerialDriver, ACLSerialRateMsg


def find_teensy():
    import serial.tools.list_ports
    for port in serial.tools.list_ports.comports():
        if port.manufacturer and port.manufacturer.lower() == 'teensyduino':
            return port.device
    return None


def getstats(data):
    hz = 1./getTs(data)
    print('Sample rate: {:.0f} Hz'.format(hz))
    print('Duration:    {:.2f} seconds'.format(data[-1,0] - data[0,0]))
    print('Mean accel:  ({:.2f}, {:.2f}, {:.2f})'.format(data[:,1].mean(), data[:,2].mean(), data[:,3].mean()))
    print('std. accel:  ({:.2f}, {:.2f}, {:.2f})'.format(data[:,1].std(), data[:,2].std(), data[:,3].std()))
    print('Mean gyro:   ({:.2f}, {:.2f}, {:.2f})'.format(data[:,4].mean(), data[:,5].mean(), data[:,6].mean()))
    print('std. gyro:   ({:.2f}, {:.2f}, {:.2f})'.format(data[:,4].std(), data[:,5].std(), data[:,6].std()))


def getTs(data):
    return np.mean(np.diff(data[:,0]))


def csvwrite(file, data):
    np.savetxt(file, data, delimiter=',')


def csvread(file):
    data = np.loadtxt(file, delimiter=',')
    return data


class batch:
    def capture(Fs, seconds=1):
        port = find_teensy()
        if port is None:
            raise ConnectionError("Could not find Teensy!")


        data = []
        def imu_cb(msg):
            data.append([msg.t_us*1e-6, msg.accel_x, msg.accel_y, msg.accel_z, msg.gyro_x, msg.gyro_y, msg.gyro_z])

        driver = ACLSerialDriver(port)
        driver.sendRate(ACLSerialRateMsg(Fs))
        time.sleep(0.1)
        driver.registerCallbackIMU(imu_cb)

        time.sleep(seconds)
        driver.unregisterCallbacks()

        return np.array(data)


    def process(data, cb):
        dataf = np.zeros_like(data)
        for i in range(data.shape[0]):
            tmp = cb(data[i,0], data[i,1:4], data[i,4:7])
            dataf[i,:] = np.hstack(tmp)

        # import ipdb; ipdb.set_trace()
            # from IPython.core.debugger import Tracer; Tracer()()

        return dataf


    def plot(data,):
        fig = plt.figure(figsize=(12,8), dpi=100)
        fig.subplots_adjust(hspace=0.3)
        ax = fig.add_subplot(211)
        ax.plot(data[:,0], data[:,1], label='X')
        ax.plot(data[:,0], data[:,2], label='Y')
        ax.plot(data[:,0], data[:,3], label='Z')
        ax.grid(); ax.legend();
        ax.set_title('Accelerometer'); ax.set_xlabel('Time [s]'); ax.set_ylabel('Acceleration [m/s/s]');
        ax = fig.add_subplot(212)
        ax.plot(data[:,0], data[:,4], label='X')
        ax.plot(data[:,0], data[:,5], label='Y')
        ax.plot(data[:,0], data[:,6], label='Z')
        ax.grid(); ax.legend();
        ax.set_title('Gyro'); ax.set_xlabel('Time [s]'); ax.set_ylabel('Angular Velocity [rad/s]');


class online:
    def liveplot(Fs, Fplot=20, window_seconds=5, seconds=60):
        port = find_teensy()
        if port is None:
            raise ConnectionError("Could not find Teensy!")


        data = []
        pdata = []
        def imu_cb(msg):
            data.append([msg.t_us*1e-6, msg.accel_x, msg.accel_y, msg.accel_z, msg.gyro_x, msg.gyro_y, msg.gyro_z])

            # for plotting
            pdata.append([msg.t_us*1e-6, msg.accel_x, msg.accel_y, msg.accel_z, msg.gyro_x, msg.gyro_y, msg.gyro_z])
            if len(pdata) > Fs * window_seconds:
                pdata.pop(0)

        driver = ACLSerialDriver(port)
        driver.sendRate(ACLSerialRateMsg(Fs))
        time.sleep(0.1)
        driver.registerCallbackIMU(imu_cb)

        # https://pyqtgraph.readthedocs.io/en/latest/plotting.html#examples
        pw = pg.plot(title="Accelerometer")
        timer = pg.QtCore.QTimer()
        Ttimer = 1./Fplot
        def update():
            if not hasattr(update,'k'):
                update.k = 0
            update.k += 1
            N = len(pdata)
            pw.plot(np.array(pdata)[0:N,0], np.array(pdata)[0:N,1], pen='y', clear=True)
            pw.plot(np.array(pdata)[0:N,0], np.array(pdata)[0:N,2], pen='r')
            pw.plot(np.array(pdata)[0:N,0], np.array(pdata)[0:N,3], pen='g')
            QtGui.QApplication.processEvents()

            if Ttimer * update.k >= seconds:
                QtGui.QApplication.closeAllWindows()

        timer.timeout.connect(update)
        timer.start(Ttimer*1e3) # ms
        QtGui.QApplication.instance().exec_()
        timer.timeout.disconnect()
        driver.unregisterCallbacks()

        return np.array(data)


    def process(Fs, cb, plotCb=None, plot_raw=True, Fplot=20, window_seconds=5, seconds=60):
        port = find_teensy()
        if port is None:
            raise ConnectionError("Could not find Teensy!")


        data = []
        pdata = []
        dataf = []
        pdataf = []
        def imu_cb(msg):
            # raw data capture
            data.append([msg.t_us*1e-6, msg.accel_x, msg.accel_y, msg.accel_z, msg.gyro_x, msg.gyro_y, msg.gyro_z])

            # send data to user's callback
            r = cb(msg.t_us*1e-6, np.array([msg.accel_x, msg.accel_y, msg.accel_z]), np.array([msg.gyro_x, msg.gyro_y, msg.gyro_z]))
            dataf.append(np.hstack(r).tolist())

            # for plotting
            if plot_raw:
                pdata.append([msg.t_us*1e-6, msg.accel_x, msg.accel_y, msg.accel_z, msg.gyro_x, msg.gyro_y, msg.gyro_z])
                if len(pdata) > Fs * window_seconds:
                    pdata.pop(0)

            if plotCb is not None:
                pdataf.append(dataf[-1])
                if len(pdataf) > Fs * window_seconds:
                    pdataf.pop(0)

        driver = ACLSerialDriver(port)
        driver.sendRate(ACLSerialRateMsg(Fs))
        time.sleep(0.1)
        driver.registerCallbackIMU(imu_cb)

        if plot_raw or plotCb is not None:
            pw = pg.plot(title="Accelerometer")
            pw2 = pg.plot(title="Processed")
            timer = pg.QtCore.QTimer()
            Ttimer = 1./Fplot
            def update():
                if not hasattr(update,'k'):
                    update.k = 0
                update.k += 1

                if Ttimer * update.k >= seconds:
                    QtGui.QApplication.closeAllWindows()
                    return

                if plot_raw:
                    N = len(pdata)
                    pw.plot(np.array(pdata)[0:N,0], np.array(pdata)[0:N,1], pen='y', clear=True)
                    pw.plot(np.array(pdata)[0:N,0], np.array(pdata)[0:N,2], pen='r')
                    pw.plot(np.array(pdata)[0:N,0], np.array(pdata)[0:N,3], pen='g')

                if plotCb is not None:
                    plotCb(pw2, pdataf)

                QtGui.QApplication.processEvents()

            timer.timeout.connect(update)
            timer.start(Ttimer*1e3) # ms
            QtGui.QApplication.instance().exec_()
            timer.timeout.disconnect()
        else:
            time.sleep(seconds)

        driver.unregisterCallbacks()

        return np.array(data), np.array(dataf)