#!/usr/bin/env python
import numpy as np
import math
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
step_size = 0.5 #distance traveled by drone in each step
threshold_force = 5
alpha= 1#weightage of theta_center vs theta_goal
temp=True
pose = PoseStamped()
lidar = LaserScan()
vel = Twist()
setpoint = PoseStamped()

initial_orientation = 0
stuck = False

def callback_lidar(msg):
    global lidar
    lidar = msg

def distance(point_1,point_2):
    return np.sqrt( (point_1[0]-point_2[0])**2 + (point_1[1]-point_2[1])**2 )

def callback_pose(msg):
    global pose
    global temp
    pose = msg
    if temp:
        temp=False
        initial_orientation = quaternion_to_euler(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w,"radians")
        initial_orientation = initial_orientation[0]


def quaternion_to_euler(x, y, z, w,type="degree"):

    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = np.arcsin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(t3, t4)
    if type=="degree":
        yaw, pitch, roll =np.degrees([yaw, pitch, roll])
        yaw=int(yaw)
        pitch=int(pitch)
        roll=int(roll)
    return [yaw, pitch, roll]

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

def gap_method():
    global pose
    global lidar
    global initial_orientation
    gap_array=[]   #This gap array will contain the polar histogram
    for i in range(360):
        gap_array.append(lidar.ranges[math.floor((512/180)*i)])

    # ---- code to find the max gap in gap_array
    dictionary = {}
    truth = True
    i=0
    values=[]
    while i<len(gap_array):
        start = 0
        end = 0
        while i<len(gap_array) and gap_array[i]==float('inf'):
            if truth:
                start=i
                truth=False
            end=i
            i+=1
        if (end-start)!=0:
            dictionary[f"{start}-{end}"]=end-start+1
        truth=True
        i+=1

    keys = list(dictionary.keys())
    values = list(dictionary.values())
    max_value = max(values)
    interval = keys[values.index(max_value)]
    max_start_index = int(interval.split("-")[0])
    max_end_index = int(interval.split("-")[1])

    d1=gap_array[max_start_index-1]
    if max_end_index < (len(gap_array)-1):
        d2=gap_array[max_end_index+1]
    else:
        i=0
        while gap_array[i]==float('inf'):
            i+=1

        d2=gap_array[i]
    dmin=min(gap_array)
    euler = quaternion_to_euler(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w,"radians")

    theta_gap=np.arccos((d1+d2*np.cos(max_start_index+max_end_index))/np.sqrt(d1**2+d2**2+2*d1*d2*np.cos(max_start_index+max_end_index)))
    theta_gap = theta_gap - (i*np.pi/180 + round(euler-initial_orientation , 3))
    theta_goal= np.arctan((args.y-pose.pose.position.y)/(args.x-pose.pose.position.x)) #angle with horizontal

    theta_final= (alpha*theta_gap/d_min + theta_goal)/(alpha/dmin+1)
    return theta_final



def oa_field(args):
    global initial_orientation
    global pose
    global lidar
    global vel
    global pub_vel
    global pub_position
    global setpoint
    global step_size
    global threshold_force
    global stuck
    f_rep_x = 0
    f_rep_y = 0
    f_rep = 0

    start = time.time()
    euler = quaternion_to_euler(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w,"radians")
    euler = euler[0]
    theta = 0
    print(len(lidar.ranges))
    for i in range(0,1024):
        if lidar.ranges[i]<d_max and lidar.ranges[i]>1:

            theta = ((180*i)/512)*np.pi/180 + round(euler-initial_orientation , 3)
            #component 1
            f_rep += ( (1/lidar.ranges[i]) - (1/d_max) ) * ((distance([args.x,args.y],[pose.pose.position.x,pose.pose.position.y])**n )/(lidar.ranges[i]**2))
            #component 2
            f_rep += (n/2)*( ((1/lidar.ranges[i]) - (1/d_max) )**2) * distance([args.x,args.y],[pose.pose.position.x,pose.pose.position.y])**(n-1)

    f_rep_x = -1*f_rep*np.cos(theta)
    f_rep_y = -1*f_rep*np.sin(theta)


    f_att_x = k_att_rep * (args.x-pose.pose.position.x)/distance([args.x,args.y],[pose.pose.position.x,pose.pose.position.y])
    f_att_y = k_att_rep * (args.y-pose.pose.position.y)/distance([args.x,args.y],[pose.pose.position.x,pose.pose.position.y])
    f_tot_x= f_rep_x + f_att_x #final resultant force
    f_tot_y= f_rep_y + f_att_y

    mag=np.sqrt(f_tot_x**2+f_tot_y**2)

    if stuck and np.sqrt(f_rep_x**2+f_rep_y**2)>np.sqrt(f_att_x**2+f_att_y**2):
        print("Continuing to use alternative algorithm")
        delta = gap_method()

    elif mag<threshold_force:
        stuck=True
        print("Minima detected, switching algorithm")
        delta = gap_method()

    else:
        stuck=False
        if f_tot_x <=0:
            delta = np.pi + math.atan(f_tot_y/f_tot_x)
        else:
            delta = math.atan(f_tot_y/f_tot_x)

    x_final = pose.pose.position.x + step_size*np.cos(delta)
    y_final = pose.pose.position.y + step_size*np.sin(delta)

    setpoint.pose.position.x = x_final
    setpoint.pose.position.y = y_final
    setpoint.pose.position.z = args.z

    pub_position.publish(setpoint)
    print("f_total_x,f_total_y = ", f_tot_x,f_tot_y," and x_final,y_final = ", x_final,y_final," stuck= ",stuck)
    # vel.linear.x = speed*f_tot_x/mag
    # vel.linear.y = speed*f_tot_y/mag
    # vel.linear.z = 0
    # if np.sqrt( (vel.linear.x)**2 + (vel.linear.y)**2 )) < threshold_vel:
    #     print("Minima detected, switching algorithm")
    #     #Trigger other algo
    # else:
    #     rospy.loginfo("f_total_x,f_total_y = ", f_tot_x,f_tot_y," and vel.x,vel.y = ", vel.linear.x,vel.linear.y)

    #     pub_vel.publish(vel)

    end = time.time()
    print("time taken to complete the loop =",end - start)



if __name__=="__main__":
    parser = argparse.ArgumentParser(description="setpoint/postition")
    parser.add_argument('-x', '--x', type=int, help="x component of point")
    parser.add_argument('-y', '--y', type=int, help="y component of point")
    parser.add_argument('-z', '--z', type=int, help="z component of point")
    parser.add_argument('-t', '--takeoff',action='store_true', help="to takeoff the drone") #If present will help the drone takeoff

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
        takeoff(args.z)
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
    # initial initialization


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
