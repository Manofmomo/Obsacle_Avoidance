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
k_rep=1
d_max=30
k_att=5
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
    f_rep_x = 0
    f_rep_y = 0
    
    start = time.time()

    for i in range(0,1024):
        if lidar.ranges[i]<d_max and lidar.ranges[i]>3:

            theta = ((180*i)/512)*np.pi/180
            si_x  = np.cos(theta)
            si_y = np.sin(theta)
            f_rep_x += ((1/lidar.ranges[i]) - (1/d_max))*si_x
            f_rep_y += ((1/lidar.ranges[i]) - (1/d_max))*si_y

    f_rep_x = -k_rep*f_rep_x
    f_rep_y = -k_rep*f_rep_y

    
    f_att_x = k_att * (args.x-pose.pose.position.x)/np.sqrt((args.x-pose.pose.position.x)**2 + (args.y-pose.pose.position.y)**2)
    f_att_y = k_att * (args.y-pose.pose.position.y)/np.sqrt((args.x-pose.pose.position.x)**2 + (args.y-pose.pose.position.y)**2)
    f_tot_x= f_rep_x + f_att_x #final resultant force 
    f_tot_y= f_rep_y + f_att_y
    
    mag=np.sqrt(f_tot_x**2+f_tot_y**2)

    
    vel.linear.x = speed*f_tot_x/mag
    vel.linear.y = speed*f_tot_y/mag
    vel.linear.z = 0

    print("f_total_x,f_total_y = ", f_tot_x,f_tot_y," and vel.x,vel.y = ", vel.linear.x,vel.linear.y)

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

 
    vel.linear.x = 0
    vel.linear.y = 0
    vel.linear.z = 0
    for i in range(10):
        pub_vel.publish(vel)         
           


    
