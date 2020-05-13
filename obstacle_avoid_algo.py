#!/usr/bin/env python
import numpy as np
import rospy
import argparse 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

#parameters
k_rep=10
d_max=30
k_att=20
speed=10

pose = PoseStamped()
lidar = LaserScan()
vel = Twist()

def callback_pose(msg):
    global pose
    pose = msg

def oa_field(args):
    global pose
    global lidar
    f_rep_x = 0
    f_rep_y = 0

    for i in range(0,1024):
        if lidar.ranges[i]<d_max and lidar.ranges[i]>2:

            if i>256 and i<768:
                theta = ((180*i)/512 - 90)*np.pi/180
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
    
    vel=Twist()
    orientation = PoseStamped()
    
    vel.linear.x = speed*f_tot_x/mag
    vel.linear.y = speed*f_tot_y/mag
    vel.linear.z = 0
    
    #calculating angle to rotate the drone by 
    ang =np.arccos(f_tot_x/mag)
    if f_tot_y > 0: 
        ang = 90 - ang #this is the angle to rotate by if clockwise is positive 
    if f_tot_y =< 0:
        if ang < np.pi/2:
            ang = 90+ ang

        else:
            ang=ang-(np.pi*3/2) # will give a negative angle as we have to rotate in anticlockwise direction

    orientation.pose.position.x = pose.pose.position.x
    orientation.pose.position.y = pose.pose.position.y
    orientation.pose.position.z = pose.pose.position.z
    orientation.pose.orientation.z = ang # i am not sure of this part 
    orientation.pose.orientation.x = 0
    orientation.pose.orientation.y = 0


    return_vals= [vel,orientation]        
    return return_vals

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
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        arr=oa_field(args)
        pub_vel.publish(arr[0])
        pub_position.publish(arr[1])
        rate.sleep()


    
