#!/usr/bin/python3
import time, sys
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui

import numpy as np
from scipy import signal
from scipy.fft import fft, fftfreq, fftshift

from acl_serial_driver import ACLSerialDriver

class IMUAnalyzer:
    # Number of samples used to calculate DFT via FFT
    FFT_SIZE = 1024

    # Window function is used since we are looking at small chunks of signal
    # as they arrive in the buffer. This makes the signal look "locally staionary".
    WINDOW = 'hann'

    # How many seconds of time-domain samples to plot
    SAMPLE_WINDOW_SEC = 5

    # Plotting frequency for time-domain signals
    SAMPLE_PLOT_FREQ_HZ = 20

    # Which sensor to analyze
    SENSOR = 'gyro' # 'accel' or 'gyro'

    def __init__(self, port):

        #
        # FFT setup
        #

        self.window = signal.windows.get_window(self.WINDOW, self.FFT_SIZE)

        # data
        self.Fs = 1
        self.last_t_us = 0
        self.t = []
        self.sensx = []
        self.sensy = []
        self.sensz = []
        self.buf_sensx = []
        self.buf_sensy = []
        self.buf_sensz = []

        #
        # Plotting setup
        #

        sens = "Accelerometer" if self.SENSOR == 'accel' else "Gyro"

        # initialize Qt gui application and window
        self.default_window_size = (1000, 800)
        self.app = pg.QtGui.QApplication([])
        self.pgwin = pg.GraphicsWindow(title="{} Analyzer".format(sens))
        self.pgwin.resize(*self.default_window_size)
        self.pgwin.setBackground('k')

        self.pw = self.pgwin.addPlot(row=0, col=0, title=sens)
        self.pw2 = self.pgwin.addPlot(row=1, col=0, title="Spectrum")

        #
        # Plotting loop
        #
        
        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self._timer_cb)
        self.timer.start(1e3/self.SAMPLE_PLOT_FREQ_HZ) # ms

        #
        # Connect to Teensy
        #

        # initialize serial communications to Teensy
        self.driver = ACLSerialDriver(port)
        time.sleep(0.1) # wait for everything to initialize

        # Connect an IMU callback that will fire when a sample arrives
        self.driver.registerCallbackIMU(self._imu_cb)

        # Block on application window
        QtGui.QApplication.instance().exec_()

        # clean up to prevent error or resource deadlock
        self.driver.unregisterCallback()

    def _timer_cb(self):
        self.pw.plot(self.t, self.sensx, pen=(1,3), clear=True)
        self.pw.plot(self.t, self.sensy, pen=(2,3))
        self.pw.plot(self.t, self.sensz, pen=(3,3))

        # always keep x-axis auto range based on SAMPLE_WINDOW_SEC
        self.pw.enableAutoRange(axis=pg.ViewBox.XAxis)


        (f, mag) = self._calcSpectrum(self.buf_sensx); self.pw2.plot(f, mag, pen=(1,3), clear=True)
        (f, mag) = self._calcSpectrum(self.buf_sensy); self.pw2.plot(f, mag, pen=(2,3))
        (f, mag) = self._calcSpectrum(self.buf_sensz); self.pw2.plot(f, mag, pen=(3,3))
        self.pw2.enableAutoRange(axis=pg.ViewBox.XAxis)

        # "drawnow"
        self.app.processEvents()

    def _imu_cb(self, msg):
        dt = (msg.t_us - self.last_t_us) * 1e-6 # us to s
        self.last_t_us = msg.t_us
        hz = 1./dt
        self.Fs = hz
        print('Got IMU at {} us ({:.0f} Hz): {:.2f}, {:.2f}, {:.2f}, \t {:.2f}, {:.2f}, {:.2f}'
                .format(msg.t_us, hz,
                        msg.accel_x, msg.accel_y, msg.accel_z,
                        msg.gyro_x, msg.gyro_y, msg.gyro_z))

        if self.SENSOR == 'accel':
            sensx = msg.accel_x
            sensy = msg.accel_y
            sensz = msg.accel_z
        else:
            sensx = msg.gyro_x
            sensy = msg.gyro_y
            sensz = msg.gyro_z

        # FIFO buffer for time-domain plotting
        self.t.append(msg.t_us)
        self.sensx.append(sensx)
        self.sensy.append(sensy)
        self.sensz.append(sensz)

        if len(self.t) > hz*self.SAMPLE_WINDOW_SEC:
            self.t.pop(0)
            self.sensx.pop(0)
            self.sensy.pop(0)
            self.sensz.pop(0)

        # FIFO buffer for FFT
        self.buf_sensx.append(sensx)
        self.buf_sensy.append(sensy)
        self.buf_sensz.append(sensz)

        if len(self.buf_sensx) > self.FFT_SIZE:
            self.buf_sensx.pop(0)
            self.buf_sensy.pop(0)
            self.buf_sensz.pop(0)


    def _calcSpectrum(self, buf):
        if len(buf) < self.FFT_SIZE:
            return [], []

        # window the data for a better behaved short-time FT style DFT
        data = np.array(buf) * self.window

        # compute DFT via FFT
        Y = fft(data)

        # make DFT look as you'd expect, plotting real part
        Y = (np.abs(fftshift(Y))/self.FFT_SIZE)

        # compute frequency bins
        f = fftshift(fftfreq(self.FFT_SIZE, 1./self.Fs))

        return f, Y

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

    analyzer = IMUAnalyzer(port)

if __name__ == '__main__':
    main()
