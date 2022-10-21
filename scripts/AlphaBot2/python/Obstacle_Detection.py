#!/usr/bin/env python3
import rospy
import RPi.GPIO as GPIO
import time
from AlphaBot2 import AlphaBot2

from Augbot.msg import synchPoint

DR = 16 #right infrared sensor
DL = 19 #left infrared sensor
BUZ = 4

def beep_on():
	GPIO.output(BUZ,GPIO.HIGH)
def beep_off():
        GPIO.output(BUZ,GPIO.LOW)

def detect():
	Ab = AlphaBot2()
	prev_time = round(time.time() * 1000)

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
			if ( DL_status == 0 ): ##left sensor detects obstacle
				t = round(time.time() * 1000)
				pub.publish ( synchPoint( t, t - prev_time ) )
				beep_on()
				time.sleep(0.01)
				beep_off()
				time.sleep(5)
				prev_time = t
			else:
				time.sleep(0.01)
	except KeyboardInterrupt:
		GPIO.cleanup()

if __name__ == '__main__':

	pub = rospy.Publisher('synchPoints', synchPoint, queue_size=1)
	rospy.init_node('obstacleDetetion', anonymous=True)
   
	detect()
