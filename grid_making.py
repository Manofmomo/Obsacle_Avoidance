#!/usr/bin/env python
import numpy as np
import rospy
import time
import argparse 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

#parameters
size_grid=10 #boxes per meter
speed=5

pose = PoseStamped()
lidar = LaserScan()
vel = Twist()

def callback_pose(msg):
    global pose
    pose = msg

def oa_field(args):
    global pose
    global lidar
    global vel
    global pub_vel
    global size_grid
    start = time.time()
    for d in range(0,d_max):
        for theta in range
    for i in range(0,1024):
        if lidar.ranges[i]<d_max and lidar.ranges[i]>3:

            theta = ((180*i)/512)*np.pi/180
            si_x  = np.cos(theta)
            si_y = np.sin(theta)
            x_index= round(si_x*lidar.ranges[i]*size_grid) + round((pose.pose.position.x)*size_grid)
            y_index= round(si_y*lidar.ranges[i]*size_grid) + round((pose.pose.position.y)*size_grid)
            
            
            grid[x_index][y_index]=100

    pub_vel.publish(vel)

    end = time.time()
    print("time taken to complete the loop =",end - start)

    
    
   

def callback_lidar(msg):
    global lidar
    lidar = msg
 


if __name__=="__main__":
    parser = argparse.ArgumentParser(description="setpoint/postition")
    parser.add_argument('-x', '--x', type=int, help="x component of point")
    parser.add_argument('-y', '--y', type=int, help="y component of point")
    parser.add_argument('-z', '--z', type=int, help="z component of point")
    args = parser.parse_args()
    print(args.x)
    print(args.y)
    print(args.z)
    

    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback_pose)
    rospy.Subscriber('/spur/laser/scan',LaserScan,callback_lidar)
    pub_vel=rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped',Twist,queue_size=20)
    pub_position=rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    rospy.init_node("OA",anonymous=True)
 
    
    pose.pose.position.z = args.z
    for i in range(0,10):
        pub_position.publish(pose)
        rospy.sleep(1/5.)
    while not pose.pose.position.z - args.z < 0.5 or pose.pose.position.z < -0.5:
        rospy.sleep(1/5.)
        
    print("-------------Height Reached------------")

    rate = rospy.Rate(2)
    d=0
    while not rospy.is_shutdown():

        oa_field(args)
        d=np.sqrt((args.x-pose.pose.position.x)**2 + (args.y - pose.pose.position.y)**2)
        if d < 0.5:
            print("----------Setpoint Reached---------- ")
            break
        rate.sleep()

    pose.pose.position.x = args.x
    pose.pose.position.y = args.y
    pose.pose.position.z = args.z
    for i in range(0,10):
        pub_position.publish(pose)
        rospy.sleep(1/5.)         
           


    
