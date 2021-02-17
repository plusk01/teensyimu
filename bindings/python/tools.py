import sys

def find_teensy():
  TEENSY_VENDOR_HEX = '16c0'
  import serial.tools.list_ports
  for port in serial.tools.list_ports.comports():
    if ((port.manufacturer and 
          port.manufacturer.lower() == 'teensyduino') or
        port.vid and port.vid == int(TEENSY_VENDOR_HEX, 16)):
      return port.device
  return None

def find_teensy_or_die():
  port = find_teensy()
  if port is None:
      print("Could not find Teensy!")
      sys.exit()
  return port
