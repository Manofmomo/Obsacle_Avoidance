#!/usr/bin/ehttps://cs231n.github.io/ nv python
import numpy as np
import rospy
import argparse 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
#parameters 
speed=4
oa_radius=20

lidar = LaserScan()
vel = Twist()
pose =PoseStamped()

pub1 = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

pub_vel=rospy.Publisher('/mavros/setpoint_velocity/cmd_vel',TwistStamped,queue_size=20)

def callback_pose(msg):
    global pose 
    pose = msg
def callback_lidar(msg):
    global lidar
    lidar =msg
    
    
def oa_detect():
    global pose
    global lidar 
    global vel
    global pub_vel
    msg = lidar
    #takes lidar values and sees if an object is detected
    counter =0
    for i in range(256,768):
        if msg.ranges[i]<oa_radius and msg.ranges[i]>1:
            print(msg.ranges[i])
            print("Angle found at",i*msg.angle_increment*180/np.pi)
            rospy.loginfo("object detected")
            counter=1
            stop_now() #stops drone 
            break
    
    if counter == 1:
        rebound() # rebound is called 

def setpoint(args):
    global pose
    pose.pose.position.x = args.x
    pose.pose.position.y = args.y
    pose.pose.position.z = args.z
    pub1.publish(pose)
    rospy.sleep(1.)
    rospy.loginfo('Setpoint added')


                
def rebound(): # finds the rebound angle by the algo
    global lidar
    msg=lidar
    rospy.loginfo('Rebound called')
    
    sum_di=0
    sum_idi=0
    j = -np.pi  #initalise index 
    for i in range(0,1024): # -135 to 135
        if msg.ranges[i]==float("inf"):
            sum_di=sum_di + 500
            sum_idi=sum_idi + j*500

        if msg.ranges[i]>msg.range_min and msg.ranges[i]<msg.range_max:
            sum_di=sum_di + msg.ranges[i]
            sum_idi=sum_idi + j*msg.ranges[i]
        j=j+msg.angle_increment

    ang=sum_idi/sum_di
    print("ANGLE REBOUNDED ",ang*180/np.pi)
    rebound_pub(ang)
    #temp for testing
    #rospy.sleep(1.)
    #oa_detect()

def rebound_pub(ang): #publishes the calculated angle
    rospy.loginfo('publishing rebound angle')
    global vel
    if ang > 0:
        ang= ang + np.pi/2
    if ang < 0:
        ang = np.pi/2 + ang
    vel.linear.x=speed*np.cos(ang)
    vel.linear.y=speed*np.sin(ang)
    vel.linear.z=0
    for i in range(4):
        pub_vel.publish(vel)
        rospy.sleep(1/5.)
    oa_detect()
    
def stop_now():  #stops the drone 
    global vel
    vel.linear.x=0
    vel.linear.y=0
    vel.linear.z=0
    rospy.loginfo('Stopping drone')
    for i in range(10):
        pub_vel.publish(vel)
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
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback_pose)
    rospy.Subscriber('/spur/laser/scan',LaserScan,callback_lidar)
    pub_vel=rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped',Twist,queue_size=20)
    pub_position=rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    rospy.init_node("OA",anonymous=True)
    r = rospy.Rate(10)
    try:
        while not rospy.is_shutdown():
            rospy.loginfo('reran loop')
            setpoint(args)
            oa_detect()
            d=np.sqrt((args.x-pose.pose.position.x)**2 + (args.y - pose.pose.position.y)**2)
            if d < 0.5:
                print("----------Setpoint Reached---------- ")
                break

            r.sleep()

    except rospy.ROSInterruptException:
        pass
