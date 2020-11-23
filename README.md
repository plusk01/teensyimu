Real-time IMU Processing from Teensy
====================================

Supporting software to stream IMU (Adafruit ICM-20948) connected to Teensy 4.0 via SPI onto desktop computer via serial. Serial IO is handled in C++ and exposed to Python with pybind11.

## Getting Started

After cloning this project onto your computer:

1. Flash Teensy with `firmware/firmware.ino` sketch
2. Build C++ driver with Python bindings (requires Boost):
  
  ```bash
  mkdir build
  cd build
  cmake ..
  make
  ```
3. Install Python bindings for system-wide use:

  ```bash
  cd build
  cd bindings/python
  pip install .
  ```

## Example

Plot IMU data in real-time by running the `plotimu_live.py` script in `examples`. Make sure to install `pyqtgraph` with `pip install pyqtgraph`.

