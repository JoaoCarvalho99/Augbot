#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
import time
from AlphaBot2 import AlphaBot2

from Augbot.msg import synchPoint

DR = 16
DL = 19
BUZ = 4

def beep_on():
	GPIO.output(BUZ,GPIO.HIGH)
def beep_off():
        GPIO.output(BUZ,GPIO.LOW)

def detect():
	Ab = AlphaBot2()

	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)
	GPIO.setup(DR,GPIO.IN,GPIO.PUD_UP)
	GPIO.setup(DL,GPIO.IN,GPIO.PUD_UP)
	GPIO.setup(BUZ,GPIO.OUT)

	try:
		while True:
			DR_status = GPIO.input(DR)
			DL_status = GPIO.input(DL)
#			if(DL_status == 0) or (DR_status == 0):
			if ( DL_status == 0 ):
				pub.publish ( synchPoint( round(time.time() * 1000) ) )
				beep_on()
				time.sleep(0.01)
				beep_off()
				time.sleep(5)
			else:
				 time.sleep(0.01)
	except KeyboardInterrupt:
		GPIO.cleanup()

if __name__ == '__main__':

	pub = rospy.Publisher('synchPoints', synchPoint, queue_size=1)
	rospy.init_node('obstacleDetetion', anonymous=True)
   
	detect()
