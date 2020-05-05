#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import LaserScan
def callback_lidar(msg):
    global lidar_range
    lidar_range = msg


def oa_detect():
    rospy.init_node("OA detector",anonymous=True)
    rospy.Subscriber('/spur/laser/scan',LaserScan,callback_lidar)
    for i in range(0,720):
        if lidar_range.ranges[i]>msg.range_min and lidar_range.range[i]<range_max:
            if lidar_range.range[i]<1:
                stop_now()
            
    rospy.spin()
                


def stop_now():
    pub_vel=rospy.Pubisher('/mavros/setpoint_velocity/cmd_vel',TwistStamped,queue_size=20)
    vel=TwistStamped()
    vel.twist.linear.x=0
    vel.twist.linear.y=0
    vel.twist.linear.z=0
    pub_vel.publish(vel)
    



if __name__=="__main__":
    try:
        while not rospy.is_shutdown():
            oa_detect()

    except rospy.RosInterruptException:
        pass
