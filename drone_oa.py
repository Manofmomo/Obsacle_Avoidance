#!/usr/bin/env python
import numpy as np
import rospy
import argparse 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
#parameters 
speed=10
oa_radius=10

pub1 = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

pub_vel=rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped',Twist,queue_size=20)

def callback_pose(msg):
    global pose
    pose = msg


def callback_lidar(msg): #takes lidar values and sees if an object is detected
    counter =0

    for i in range(0,720):
        if msg.ranges[i]<oa_radius and msg.ranges[i]>1:
            print(msg.ranges[i])
            rospy.loginfo("object detected")
            counter=1
            stop_now() #stops drone 
            break
    
    if counter == 1:
        rebound(msg) # rebound is called 

def setpoint(args):
    point = PoseStamped()
    point.pose.position.x = args.x
    point.pose.position.y = args.y
    point.pose.position.z = args.z
    for i in range(0,10):
        pub1.publish(point)
        rospy.sleep(1/5.)

    rospy.loginfo('Setpoint added')
    


def oa_detect():   # connects to subscriber 
    rospy.Subscriber('/spur/laser/scan',LaserScan,callback_lidar)
    print('inside OA detect')

                
def rebound(msg): # finds the rebound angle by the algo
    rospy.loginfo('Rebound called')

    a0 = np.pi/512
    ar1 = 0
    di = 0
    
    for i in range(256,768):
        if (msg.ranges[i]=="inf"):
            msg.ranges[i] = 100
        ai = (i-512)*a0
        ar1 = ar1 + ai*msg.ranges[i]
        di = di +msg.ranges[i]
    
    ar = ar1/di
    rebound_pub(ar)


def rebound_pub(ang): #publishes the calculated angle
    rospy.loginfo('publishing rebound angle')
    global pose
    vel=Twist()
    orientation = PoseStamped()
    
    if (ang>0):

        vel.linear.x = 5*np.sin(ang)
        vel.linear.y = 5*np.cos(ang)
        vel.linear.z = 0

        orientation.pose.position.x = pose.pose.position.x
        orientation.pose.position.y = pose.pose.position.y
        orientation.pose.position.z = 3
        orientation.pose.orientation.z = -ang
        orientation.pose.orientation.x = 0
        orientation.pose.orientation.y = 0

        for i in range(0,10):
            pub1.publish(orientation)
            rospy.sleep(1/5.)
        for i in range(0,10):
            pub_vel.publish(vel)
            rospy.sleep(1/5.)

    if (ang<0):
        ang = -ang
        vel.linear.x = -5*np.sin(ang)
        vel.linear.y = 5*np.cos(ang)
        vel.linear.z = 0

        orientation.pose.position.x = pose.pose.position.x
        orientation.pose.position.y = pose.pose.position.y
        orientation.pose.position.z = 3
        orientation.pose.orientation.z = ang
        orientation.pose.orientation.x = 0
        orientation.pose.orientation.y = 0

        for i in range(0,10):
            pub1.publish(orientation)
            pub_vel.publish(vel)
            rospy.sleep(1/5.)


def stop_now():  #stops the drone 
    global pose
    vel = Twist()
    vel.linear.x=0
    vel.linear.y=0
    vel.linear.z=0
    rospy.loginfo('Stopping drone')
    for i in range(0,10):
        pub_vel.publish(vel)
        pub1.publish(pose)
        rospy.sleep(1/5.)



if __name__=="__main__":
    parser = argparse.ArgumentParser(description="setpoint/postition")
    parser.add_argument('-x', '--x', type=int, help="x component of point")
    parser.add_argument('-y', '--y', type=int, help="y component of point")
    parser.add_argument('-z', '--z', type=int, help="z component of point")
    args = parser.parse_args()
    print(args.x)
    print(args.y)
    print(args.z)
    rospy.init_node("OA",anonymous=True)
    r = rospy.Rate(10)
    try:

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback_pose)
        for i in range(0,100000):
            continue
        rospy.Subscriber('/spur/laser/scan',LaserScan,callback_lidar)
        setpoint(args)
        rospy.spin()
        

    except rospy.ROSInterruptException:
        pass
