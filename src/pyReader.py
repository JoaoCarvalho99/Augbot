import rospy
from serial import Serial

class pyReader():
    device = "/dev/ttyACM0"
    serialPort = Serial(device, baudrate=115200, timeout=3.0)

    while True:
      serialBytes = serialPort.readline()
      serialString = serialBytes.decode('Ascii')
      print(serialString)