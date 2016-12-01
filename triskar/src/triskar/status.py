#!/usr/bin/env python
# license removed for brevity
import rospy
import RPi.GPIO as GPIO
import os.path
#from std_msgs.msg import String

ON = 11
SERIAL_UP = 13
OTHER = 15

def statusNode():
	#Init node
	rospy.init_node('status', anonymous=True)
	
	#Init GPIO
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(ON, GPIO.OUT)
	GPIO.setup(SERIAL_UP, GPIO.OUT)
	GPIO.setup(OTHER, GPIO.OUT)
	
	#Enable ON led, disable others
	GPIO.output(ON, True)
	GPIO.output(SERIAL_UP, False)
	GPIO.output(OTHER, False)
	
	rate = rospy.Rate(1) # 1hz
	
	serialDevice = '/dev/ttyACM0'
	
	while not rospy.is_shutdown():
		if os.path.isfile(serialDevice):
			GPIO.output(SERIAL_UP, True)
		else:
			GPIO.output(SERIAL_UP, False)
	rate.sleep()

if __name__ == '__main__':
	try:
		statusNode()
		        
	except rospy.ROSInterruptException:
		pass
