#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import rospy
import RPi.GPIO as GPIO
from AlphaBot2 import AlphaBot2
from rpi_ws281x import Adafruit_NeoPixel, Color
from TRSensors import TRSensor
import time

from Augbot.msg import synchPoint


def move():

	Button = 7

	sleeptime = 0.05

	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)
	GPIO.setup(Button,GPIO.IN,GPIO.PUD_UP)

	# LED strip configuration:
	LED_COUNT      = 4      # Number of LED pixels.
	LED_PIN        = 18      # GPIO pin connected to the pixels (must support PWM!).
	LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
	LED_DMA        = 5       # DMA channel to use for generating signal (try 5)
	LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
	LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)	

	#update for reference points
	DR = 16
	DL = 19

	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)
	GPIO.setup(DR,GPIO.IN,GPIO.PUD_UP)
	GPIO.setup(DL,GPIO.IN,GPIO.PUD_UP)
	#end of reference points

	maximum = 50
	j = 0
	integral = 0
	last_proportional = 0

	def Wheel(pos):
		"""Generate rainbow colors across 0-255 positions."""
		pos = int( pos )
		if pos < 85:
			return Color(pos * 3, 255 - pos * 3, 0)
		elif pos < 170:
			pos -= 85
			return Color(255 - pos * 3, 0, pos * 3)
		else:
			pos -= 170
			return Color( 0,pos * 3, 255 - pos * 3)

	# Create NeoPixel object with appropriate configuration.
	strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS)
	# Intialize the library (must be called once before other functions).
	strip.begin()
	strip.setPixelColor(0, Color(100, 0, 0))       #Red
	strip.setPixelColor(1, Color(0, 100, 0))       #Blue
	strip.setPixelColor(2, Color(0, 0, 100))       #Green
	strip.setPixelColor(3, Color(100, 100, 0))     #Yellow
	strip.show()

	TR = TRSensor()
	Ab = AlphaBot2()
	Ab.stop()
	print("Line follow Example")
	time.sleep(0.5)
	for i in range(0,1000):
		if(i<250 or i>= 750):
			Ab.right()
			Ab.setPWMA(60)
			Ab.setPWMB(60)
		else:
			Ab.left()
			Ab.setPWMA(60)
			Ab.setPWMB(60)
		TR.calibrate()
	Ab.stop()
	print(TR.calibratedMin)
	print(TR.calibratedMax)
	while (GPIO.input(Button) != 0):
		position,Sensors = TR.readLine()
		#print(position,Sensors)
		print ( position )
		print ( Sensors )
		print (" -------------------------" )
		time.sleep( sleeptime )
	#print ("forward")
	Ab.forward()
	Ab.setPWMA(20)
	Ab.setPWMB(20)

	LEFT = 250
	RIGHT = 3750


	stopped = 0
	i = 0

	while True:
		#reference point
		DR_status = GPIO.input(DR)
		DL_status = GPIO.input(DL)
		if DL_status == 0 or DR_status == 0:
			pub.publish ( synchPoint( round(time.time() * 1000) ) )

		#movement
		position,Sensors = TR.readLine()
		if(Sensors[0] >950 and Sensors[1] >950 and Sensors[2] >950 and Sensors[3] >950 and Sensors[4] >950): #stop
			Ab.stop()
			stopped = 1
		elif (Sensors[0] <25 and Sensors[1] < 25 and Sensors[2] < 25 and Sensors[3] < 25 and Sensors[4] < 25): #backwards
			Ab.stop()
			Ab.backward()
			time.sleep( sleeptime )
			stopped = 1
		else:
			#print ( position )
			#print (Sensors)
			if ( position > LEFT and position < RIGHT ):
				if ( stopped == 1):
					stopped = 0
					Ab.forward()
					sleeptime = 0.03
			elif ( position >= RIGHT ): #move right
				Ab.right()
				sleeptime = 0.01
				stopped = 1
			elif ( position <= LEFT ): #move left
				Ab.left()
				sleeptime = 0.01
				stopped = 1
			time.sleep( sleeptime )


if __name__ == '__main__':
    initial_guess = [1.0, 1.0, 0.0]

    pub = rospy.Publisher('synchPoints', synchPoint, queue_size=1)
    rospy.init_node('Movement', anonymous=True)
   
    move()