import numpy as np
import argparse
from math import sqrt,floor
from geometry_msgs.msg import PoseStamped
import rospy
import time
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

pose = PoseStamped()
size_grid=0
maze_shape=[0,0]
map_msg = OccupancyGrid()
cost = 1


# I am assuming each box in our grid to be about 1m^3

# cost has to be the real world distance
class Node:

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0
    def __eq__(self, other):
        return self.position == other.position



def maze_node_to_pose(pose_drone,node_to_convert,maze_shape,param="object"):
    global size_grid
    x,y = pose_drone

    if param=="object":
        x_grid,y_grid = node_to_convert.position
    elif param=="list":
        x_grid,y_grid = node_to_convert

    x_max,y_max=maze_shape
    return [ ((y_grid-y_max//2)/size_grid)+x ,((x_max//2-x_grid)/size_grid)+y ]
    # check is x y is wrong



def distance(point_1,point_2):
    return np.sqrt( (point_1[0]-point_2[0])**2 + (point_1[1]-point_2[1])**2 )

def angle_calculator(point_1,point_2,point_3): #angle between 1-2 and 2-3
    a = np.array(point_1)
    b = np.array(point_2)
    c = np.array(point_3)

    ba = a - b
    bc = c - b

    cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    angle = np.arccos(cosine_angle)

    return np.degrees(angle)

def setpoint_publisher(path,pose_drone):
    setpoint = PoseStamped()
    global height
    i=0
    time_start = time.time()
    while i <len(path):
        if (time_start+10)< time.time():
            print("10s over rescanning")
            break
        grid_coordinates_1 = [path[i][0], path[i][1]]
        real_coordinates_1 = maze_node_to_pose(pose_drone,grid_coordinates_1,maze_shape,"list")
        Handle=True
        flag=0
        while Handle and i <len(path):
            grid_coordinates_2 = [path[i+1][0], path[i+1][1]]
            real_coordinates_2 = maze_node_to_pose(pose_drone,grid_coordinates_2,maze_shape,"list")

            if angle_calculator(real_coordinates_1,pose_drone,real_coordinates_2)<10:
                i+=1
                flag=1
            else:
                Handle=False
                break

        if flag==1:
            point_to_publish= real_coordinates_2
        else:
            point_to_publish= real_coordinates_1

        setpoint.pose.position.x = point_to_publish[0]
        setpoint.pose.position.y = point_to_publish[1]
        setpoint.pose.position.z = height
        # add code to publish point
        pub_position.publish(setpoint)
        while distance([pose.pose.position.x,pose.pose.position.y],point_to_publish)>2:
            rospy.sleep(1/5.)
        i+=1
        #loop continues

def return_path(current_node,maze):
    path = []

    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    # Return reversed path as we need to show from start to end path
    path = path[::-1]

    return path



def search(maze, cost,end,pose_drone): # end will be the real world coordinates and pose drone will be [x,y]
    no_rows, no_columns = np.shape(maze)

    start= [no_rows//2,no_columns//2]
    start_node = Node(None, tuple(start))
    start_node.g = start_node.h = start_node.f = 0

    yet_to_visit_list = visited_list = []

    yet_to_visit_list.append(start_node)

    outer_iterations = 0
    max_iterations = 1000

    move  =  [[-1, 0 ],[-1, -1 ],[-1, 1 ],[1, -1 ],[1, 1 ],[ 0, -1],[ 1, 0 ], [ 0, 1 ]]

    while len(yet_to_visit_list) > 0:
        outer_iterations += 1

        current_node = yet_to_visit_list[0]
        current_index = 0
        for index, item in enumerate(yet_to_visit_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index

        if outer_iterations > max_iterations:
            print ("Finished exploring")
            return return_path(current_node,maze)

        yet_to_visit_list.pop(current_index)
        visited_list.append(current_node)


        if distance(maze_node_to_pose(pose_drone,current_node,np.shape(maze)) ,end) < 2:
            print("Found the end point")
            return return_path(current_node,maze)

        children = []

        for new_position in move:

            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            if (node_position[0] > (no_rows - 1) or
                node_position[0] < 0 or
                node_position[1] > (no_columns -1) or
                node_position[1] < 0):
                continue

            if maze[node_position[0]][node_position[1]] != 0:
                continue

            new_node = Node(current_node, node_position)
            children.append(new_node)

        for child in children:

            if len([visited_child for visited_child in visited_list if visited_child == child]) > 0:
                continue

            child.g = current_node.g + cost
            child.h = distance(maze_node_to_pose(pose_drone,child,np.shape(maze)) ,end)
            child.h = child.h**2 # this ensures that our drone doesn't get stuck in the middle
            child.f = child.g + child.h

            if len([i for i in yet_to_visit_list if child == i and child.g > i.g]) > 0:
                continue
            yet_to_visit_list.append(child)


def callback_pose(msg):
    global pose
    pose = msg

def callback_map(msg):
    global map_msg
    map_msg = msg



if __name__ == '__main__':

    global size_grid
    global maze_shape

    parser = argparse.ArgumentParser(description="setpoint/postition")
    parser.add_argument('-x', '--x', type=int, help="x component of point")
    parser.add_argument('-y', '--y', type=int, help="y component of point")
    parser.add_argument('-z', '--z', type=int, help="z component of point")
    args =parser.parse_args()
    print(args.x)
    print(args.y)
    print(args.z)

    rospy.init_node("a_star",anonymous=True)
    pub_position=rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback_pose)

    height = args.z

    pose.pose.position.z = args.z
    for i in range(0,10):
        pub_position.publish(pose)
        rospy.sleep(1/5.)
    while not pose.pose.position.z - args.z < 0.5 or pose.pose.position.z < -0.5:
        rospy.sleep(1/5.)

    print("-------------Height Reached------------")
    end=[ args.x , args.y ]

    rate = rospy.Rate(2)
    rospy.Subscriber('/map_handle', OccupancyGrid, callback_map)

    while not rospy.is_shutdown():
        grid=map_msg.data
        np.reshape(grid,(map_msg.info.height,map_msg.info.width))
        pose_drone = [pose.pose.position.x , pose.pose.position.y]
        size_grid=1/map_msg.info.resolution

        no_rows, no_columns = np.shape(grid)
        maze_shape=[no_rows, no_columns]

        path = search(grid, cost ,end , pose_drone)
        setpoint_publisher(path,pose_drone)
