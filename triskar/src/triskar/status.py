#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
import os.path
from std_msgs.msg import String
from sensor_msgs.msg import Joy
#from record_ros.srv import String_cmd

ON = 11
SERIAL_UP = 13
RECORDING = 15

isRecording = False

def joyCallback(data):
	if data.buttons[9] == 1:
		rospy.wait_for_service('/recorder/cmd')
		try:
#			service = rospy.ServiceProxy('/recorder/cmd', String_cmd)
			if not isRecording:
				service('record')
				isRecording = True
				GPIO.output(RECORDING, True)
			else:
				service('stop')
			
		except rospyServiceException, e:
			isRecording = False
			GPIO.output(RECORDING, False)


def statusNode():
	#Init node
	rospy.init_node('status', anonymous=True)
	
	#Setup subscriber 
	rospy.Subscriber('/joy', Joy, joyCallback);
	
	#Init GPIO
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(ON, GPIO.OUT)
	GPIO.setup(SERIAL_UP, GPIO.OUT)
	GPIO.setup(RECORDING, GPIO.OUT)
	
	#Enable ON led, disable others
	GPIO.output(ON, True)
	GPIO.output(SERIAL_UP, False)
	GPIO.output(RECORDING, False)
	
	rate = rospy.Rate(1) # 1hz
	
	serialDevice = '/dev/ttyACM0'
	
	while not rospy.is_shutdown():
		if os.path.exists(serialDevice):
			GPIO.output(SERIAL_UP, True)
		else:
			GPIO.output(SERIAL_UP, False)
		rate.sleep()
			

if __name__ == '__main__':
	try:
		statusNode()
		        
	except rospy.ROSInterruptException:
		pass
	GPIO.output(ON, False)
	GPIO.output(SERIAL_UP, False)
        GPIO.output(RECORDING, False)
	GPIO.cleanup()
