#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
def callback_lidar(msg):
    counter =0 
    for i in range(0,720):
        if msg.ranges[i]>msg.range_min and msg.ranges[i]<msg.range_max:
            if msg.ranges[i]<10:
                rospy.loginfo("object detected")
                counter=1
                stop_now()
    if counter == 0:
        print("All Clear")



def oa_detect():
    rospy.init_node("OA",anonymous=True)
    rospy.Subscriber('/spur/laser/scan',LaserScan,callback_lidar)
    rospy.spin()
                


def stop_now():
    pub_vel=rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',TwistStamped,queue_size=20)
    vel=TwistStamped()
    vel.twist.linear.x=0
    vel.twist.linear.y=0
    vel.twist.linear.z=0
    pub_vel.publish(vel)
    



if __name__=="__main__":
    try:
        while not rospy.is_shutdown():
            oa_detect()

    except rospy.ROSInterruptException:
        pass
