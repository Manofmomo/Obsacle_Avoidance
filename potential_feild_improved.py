#!/usr/bin/env python
import numpy as np
import rospy
import time
import argparse
from dronekit import connect
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import SetMode

#parameters
d_max=30
k_att_rep=5 #ratio of attractive force to repulsive force
speed=5 #Max speed of drone
threshold_vel=0.5  #minimum speed at which to declare minima , ranges from 0 to speed
n=4 #paramater decreasing repulsive

pose = PoseStamped()
lidar = LaserScan()
vel = Twist()


def callback_lidar(msg):
    global lidar
    lidar = msg

def distance(point_1,point_2):
    return np.sqrt( (point_1[0]-point_2[0])**2 + (point_1[1]-point_2[1])**2 )

def callback_pose(msg):
    global pose
    pose = msg

def takeoff(height):
    vehicle =connect('udp:127.0.0.1:14551',wait_ready=True)
    print("Performing prearm checks")
    while not vehicle.is_armable:
        print("Waiting for drone...")
        rospy.sleep(1.)

    mode_change = rospy.ServiceProxy('mavros/set_mode', SetMode)

    mode_change.call(custom_mode='GUIDED')

    rospy.loginfo('Mode Guided')
    arming= rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    arming.call(True)

    taking_off =rospy.ServiceProxy('mavros/cmd/takeoff', CommandTOL)
    taking_off.call(altitude=height)
    rospy.sleep(1.)


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
            #component 1
            f_rep += ( (1/lidar.ranges[i]) - (1/d_max) ) * (distance([args.x,args.y],[pose.pose.position.x,pose.pose.position.y])**n )/(lidar.ranges[i]**2)
            #component 2
            f_rep += (n/2)*( ((1/lidar.ranges[i]) - (1/d_max) )**2) * distance([args.x,args.y],[pose.pose.position.x,pose.pose.position.y])**(n-1)

    f_rep_x = -1*f_rep*np.cos(theta)
    f_rep_y = -1*f_rep*np.sin(theta)


    f_att_x = k_att * (args.x-pose.pose.position.x)/distance([args.x,args.y],[pose.pose.position.x,pose.pose.position.y])
    f_att_y = k_att * (args.y-pose.pose.position.y)/distance([args.x,args.y],[pose.pose.position.x,pose.pose.position.y])
    f_tot_x= f_rep_x + f_att_x #final resultant force
    f_tot_y= f_rep_y + f_att_y

    mag=np.sqrt(f_tot_x**2+f_tot_y**2)

    vel.linear.x = speed*f_tot_x/mag
    vel.linear.y = speed*f_tot_y/mag
    vel.linear.z = 0
    if np.sqrt( (vel.linear.x)**2 + (vel.linear.y)**2 )) < threshold_vel:
        print("Minima detected, switching algorithm")
        #Trigger other algo
    else:
        rospy.loginfo("f_total_x,f_total_y = ", f_tot_x,f_tot_y," and vel.x,vel.y = ", vel.linear.x,vel.linear.y)

        pub_vel.publish(vel)

    end = time.time()
    print("time taken to complete the loop =",end - start)



if __name__=="__main__":
    parser = argparse.ArgumentParser(description="setpoint/postition")
    parser.add_argument('-x', '--x', type=int, help="x component of point")
    parser.add_argument('-y', '--y', type=int, help="y component of point")
    parser.add_argument('-z', '--z', type=int, help="z component of point")
    parser.add_argument(''-t', '--takeoff',action='store_true', help="to takeoff the drone") #If present will help the drone takeoff

    args = parser.parse_args()
    print(args.x)
    print(args.y)
    print(args.z)

    rospy.Subscriber('/mavros/local_position/pose',PoseStamped, callback_pose)
    rospy.Subscriber('/spur/laser/scan',LaserScan,callback_lidar)

    pub_vel=rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped',Twist,queue_size=20)
    pub_position=rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    rospy.init_node("OA",anonymous=True)

    if args.takeoff:
        takeoff(args.height)
    else:
        pose.pose.position.z = args.z
        for i in range(0,10):
            pub_position.publish(pose)
            rospy.sleep(1/5.)
        while not  pose.pose.position.z - args.z < 0.5 or pose.pose.position.z - args.z < -0.5:
            rospy.sleep(1/5.)

        print("-------------Height Reached------------")

    rate = rospy.Rate(2)
    d=0

    while not rospy.is_shutdown():

        oa_field(args)
        d= distance([args.x,args.y],[pose.pose.position.x,pose.pose.position.y])
        if d < 1:
            break
        rate.sleep()

    #Final publish to reach exact coordinates
    pose.pose.position.x = args.x
    pose.pose.position.y = args.y
    pose.pose.position.z = args.z
    for i in range(0,10):
        pub_position.publish(pose)
        rospy.sleep(1/5.)

    print("----------Setpoint Reached---------- ")
