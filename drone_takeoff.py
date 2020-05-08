#!/usr/bin/env python 

import rospy

from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import SetMode

def takeoff(height):
	rospy.init_node('drone_udd',anonymous=True)
	mode_change = rospy.ServiceProxy('mavros/set_mode', SetMode)
	mode_change.call(custom_mode='GUIDED')
	
	rospy.sleep(4.)

	rospy.loginfo('Mode Guided')
	arming= rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
	arming.call(True)

	taking_off =rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
	taking_off.call(altitude=height)
	rospy.spin()

takeoff(int(input("Enter height to takeoff to")))
