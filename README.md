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

### Windows Build

Compilation of this package has been tested on a Windows 10 machine. The environment was setup as follows. Explicit versions used are listed, but these steps are expected to work with reasonably recent versions of these tools.

1. Install [git bash](https://git-scm.com/downloads) (*2.30.1*)
2. Install [CMake](https://cmake.org/download/) (*3.19.5*)
3. Install [Python](https://www.python.org/) (*3.9.1*)
4. Install [Visual Studio Community](https://visualstudio.microsoft.com/vs/community/) or Professional (*Pro 2019, v16.8.5* with *MSVC 14.28.29333*)
5. Install [Boost](https://sourceforge.net/projects/boost/files/boost-binaries) (*1.75.0*, [`boost_1_75_0-msvc-14.2-64.exe`](https://sourceforge.net/projects/boost/files/boost-binaries/1.75.0/))

Once the development environment is setup, use `git bash` (or `cmd`) to run the following commands

```bash
$ git clone https://github.com/plusk01/teensyimu # clone this repo in your preferred directory
$ cd teensyimu
$ mkdir build
$ cd build
$ cmake -DBUILD_SHARED_LIBS=OFF ..
$ cmake --build . --target ALL_BUILD --config Release # or open in VS: start teensyimu.sln
```

Once the package builds successfully, you can install the `teensyimu` Python package as described below.

### Installing Python Package

This repo provides the `teensytools` Python package to allow easy access to the hardware from Python.

## Example

Plot IMU data in real-time by running the `plotimu_live.py` script in `examples`. Make sure to install `pyqtgraph` with `pip install pyqtgraph`.

