#!/usr/bin/ehttps://cs231n.github.io/ nv python
import numpy as np
import rospy
import argparse 
import time
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
#parameters 
speed=5
oa_radius=10
counter = 0
pose = PoseStamped()
lidar = LaserScan()

pub1 = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
pub_vel=rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped',Twist,queue_size=20)

def callback_pose(msg):
    global pose
    pose = msg


def callback_lidar(msg): #takes lidar values and sees if an object is detected
    global counter
    global lidar

    lidar = msg
    

def setpoint(args):
    global pose
    k_att = 5
    speed =5
    vel = Twist()

    f_att_x = k_att * (args.x-pose.pose.position.x)/np.sqrt((args.x-pose.pose.position.x)**2 + (args.y-pose.pose.position.y)**2)
    f_att_y = k_att * (args.y-pose.pose.position.y)/np.sqrt((args.x-pose.pose.position.x)**2 + (args.y-pose.pose.position.y)**2)
    mag=np.sqrt(f_att_x**2+f_att_y**2)

    vel.linear.x = speed*f_att_x/mag
    vel.linear.y = speed*f_att_y/mag

    pub_vel.publish(vel)
    rospy.loginfo('Setpoint added')
    

                
def stop_now():  #stops the drone 
    global pose
    
    rospy.loginfo('Stopping drone')
    vel = Twist()
    vel.linear.x = 0
    vel.linear.y = 0
    vel.linear.z = 0

    pub_vel.publish(vel)
    
     
def main_function():
    global lidar
    global pose
    global counter
    global args
    start = time.time()
    counter=0 
    for i in range(1,512):
        if lidar.ranges[i]<15 and lidar.ranges[i]>1:
            print(lidar.ranges[i])
            rospy.loginfo("object detected")
            counter=1
            break

    
    if counter:
        rospy.loginfo("executing OA")
        stop_now() #stops drone 
        a0 = np.pi/512
        ar1 = 0
        di = 0
    
        for i in range(0,1024):
            if (lidar.ranges[i]=="inf"):
                lidar.ranges[i] = 100
            ai = (i-512)*a0
            ar1 = ar1 + ai*lidar.ranges[i]
            di = di +lidar.ranges[i]
    
        ar = ar1/di
        print("-----------------------------",ar)
        
        vel = Twist()
        vel.linear.x = speed*math.sin(ar)
        vel.linear.y = speed*math.cos(ar)
        vel.linear.z = 0

    

        orientation = PoseStamped()
        orientation.pose.position.x = pose.pose.position.x
        orientation.pose.position.y = pose.pose.position.y
        orientation.pose.position.z = pose.pose.position.z
        orientation.pose.orientation.z = 0
        orientation.pose.orientation.x = 0
        orientation.pose.orientation.y = 0

        pub1.publish(orientation)
        time.sleep(1)
        pub_vel.publish(vel)


    elif counter==0:
        setpoint(args)
    
    end = time.time()
    rospy.loginfo(f"total time take to complete the main_function = {end-start}")





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
    rate = rospy.Rate(1/2)
    try:

        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback_pose)
        for i in range(0,100000):
            continue
        rospy.Subscriber('/spur/laser/scan',LaserScan,callback_lidar)
        time.sleep(2)
        while not rospy.is_shutdown():
            main_function()
            rate.sleep()


        

    except rospy.ROSInterruptException:
        pass
