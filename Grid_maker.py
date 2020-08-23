#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from math import ceil,sqrt,floor
import time
import tf

lidar = LaserScan()
pose = PoseStamped()
map_msg = OccupancyGrid()

pub_grid= rospy.Publisher('map_handle',OccupancyGrid,queue_size=10)
size_grid = 5 #boxes per meter
d_max = 30
radius=1
truth = True
initial_orientation = None


def callback_pose(msg):
    global pose
    global truth
    global initial_orientation

    pose = msg
    #rospy.loginfo("INSIDE CALLBACK_POSE")
    if truth:
        initial_orientation = tf.transformations.euler_from_quaternion([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

        initial_orientation = initial_orientation[2]
        truth = False



def callback_lidar(msg):
    global lidar
    lidar = msg
    global size_grid
    global map_msg
    global initial_orientation
    global pose
    grid=[]
    grid = np.ndarray((2*size_grid*d_max+1,2*size_grid*d_max+1), buffer=np.zeros((2*size_grid*d_max+1,2*size_grid*d_max+1), dtype=np.int),dtype=np.int)
   # grid = [[0 for i in range(0,size_grid*d_max+1)] for j in range(0,size_grid*d_max+1)]

    off_x = (size_grid*d_max)
    off_y = (size_grid*d_max)
    euler = tf.transformations.euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])

    euler = euler[2]


    for i in range(0,1024):
        if lidar.ranges[i]<d_max and lidar.ranges[i]>2:

            theta = (np.pi*i)/512 + round(euler-initial_orientation , 3)

            y = lidar.ranges[i]*np.cos(theta)
            x = np.sin(theta)*lidar.ranges[i]
            #print(y,x)

            x_index = int(off_x - round(x*size_grid))
            y_index = int(off_y + round(y*size_grid))
            grid[x_index,y_index] = 100
            #print(y_index,x_index)
            for offset_y in range(ceil(-radius*size_grid),floor(size_grid*radius)):
                offset_x_range=floor(sqrt((radius*size_grid)**2-offset_y**2))

                for offset_x in range(-offset_x_range,offset_x_range):
                    try:
                        if grid[x_index+offset_x,y_index+offset_y]<ceil( 100- (sqrt(offset_x**2+offset_y**2)/(radius*size_grid))*100 ):
                            grid[x_index+offset_x,y_index+offset_y]=ceil( 100- (sqrt(offset_x**2+offset_y**2)/(radius*size_grid))*100 )

                    except:
                        continue


    map_msg.header.frame_id = "map"
    map_msg.header.stamp = rospy.Time.now()
    map_msg.info.map_load_time = rospy.Time.now()
    print(map_msg.info.map_load_time)
    map_msg.info.resolution = 1/size_grid
    print(map_msg.info.resolution)
    map_msg.info.width = 2*size_grid*d_max + 1
    map_msg.info.height = 2*size_grid*d_max + 1
    map_msg.info.origin.position.x = pose.pose.position.x-d_max#I dont know what to enter here
    map_msg.info.origin.position.y = pose.pose.position.y+d_max#I dont know what to enter here
    map_msg.data = []
    for i in range(0, 2*size_grid*d_max + 1):
        for j in range(0, 2*size_grid*d_max + 1):
            map_msg.data.append(grid[i,j])

    pub_grid.publish(map_msg)
    # for i in range(0, (size_grid*d_max + 1)**2):
    #     if map_msg.data[i]==100:
    #         print("yes")

    rospy.loginfo("Map Updated")




if __name__=="__main__":

    rospy.init_node('Grid_maker')
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, callback_pose)
    time.sleep(1.5)
    print("completed callback")
    rospy.Subscriber("/spur/laser/scan", LaserScan, callback_lidar)


    print("hi")
    rospy.spin()
