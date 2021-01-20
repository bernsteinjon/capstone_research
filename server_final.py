import time
import zmq
import simplejson as json
import numpy as np
import rospy
import math
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import TurtlebotLidar
from nav_msgs.msg import Odometry
from roboviz import MapVisualizer
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

# Connect to script getting raw data
context = zmq.Context()
socket1 = context.socket(zmq.REP)
socket1.bind("tcp://*:5555")
# Connect to teleop
socket2 = context.socket(zmq.REQ)
socket2.bind("tcp://*:5560")

rospy.init_node("speed_controller")

speed = Twist()
r = rospy.Rate(4)

# Declare map size
MAP_SIZE_PIXELS = 50
MAP_SIZE_METERS = 10

start_x = MAP_SIZE_PIXELS/2
start_y = MAP_SIZE_PIXELS/2

# Create an RMHC SLAM object with a laser model and optional robot model
slam = RMHC_SLAM(TurtlebotLidar(), MAP_SIZE_PIXELS, MAP_SIZE_METERS, random_seed=9999)
# Set up a SLAM display
viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, 'SLAM')
# Initialize empty map
mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

x = 0.0
y = 0.0
theta = 0.0

# epsilon for testing whether a number is close to zero
_EPS = np.finfo(float).eps * 4.0

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]


# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

def euler_from_matrix(matrix, axes='sxyz'):
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _ = _TUPLE2AXES[axes]
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    M = np.array(matrix, dtype=np.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
        if sy > _EPS:
            ax = math.atan2( M[i, j],  M[i, k])
            ay = math.atan2( sy,       M[i, i])
            az = math.atan2( M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2( sy,       M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
        if cy > _EPS:
            ax = math.atan2( M[k, j],  M[k, k])
            ay = math.atan2(-M[k, i],  cy)
            az = math.atan2( M[j, i],  M[i, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2(-M[k, i],  cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az


def quaternion_matrix(quaternion):
    q = np.array(quaternion[:4], dtype=np.float64, copy=True)
    nq = np.dot(q, q)
    if nq < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / nq)
    q = np.outer(q, q)
    return np.array((
        (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3], 0.0),
        (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3], 0.0),
        (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1], 0.0),
        (                0.0,                 0.0,                 0.0, 1.0)
        ), dtype=np.float64)



def euler_from_quaternion(quaternion, axes='sxyz'):
    return euler_from_matrix(quaternion_matrix(quaternion), axes)

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

goal = Point()
sub = rospy.Subscriber("/odom", Odometry, newOdom)
pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

buffer_counter2 = 0

def moveRobot(move_x, move_y):

    test = 0
    goal.x = x + move_x
    goal.y = y + move_y
    print("Moving from: " + str(x), str(y) + " to: " + str(goal.x), str(goal.y))
    while test != 1:
        inc_x = goal.x - x
        inc_y = goal.y - y
        angle_to_goal = atan2(inc_y, inc_x)
        if abs(angle_to_goal - theta) > 0.1:
            speed.linear.x = 0.0
            speed.angular.z = 0.3
        else:
            speed.linear.x = 0.5
            speed.linear.z = 0.0

        pub.publish(speed)


        # 0.8 @ 35,35 was promising

        if ((goal.x < x) & ((x - goal.x) < .09)):
            #print("reached goal")
            test = 1
        elif ((x < goal.x) & ((goal.x - x) < .09)):
            #print("reached goal")
            test = 1

        r.sleep()
    return 0

class Node():
    """A node class for A* Pathfinding"""

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

    def __hash__(self):               #<-- added a hash method
        return hash(self.position)


def astar(maze, start, end):
    """Returns a list of tuples as a path from the given start to the given end in the given maze"""

    # Create start and end node
    start_node = Node(None, start)
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both open and closed list
    open_list = []
    closed_list = set()                # <-- closed_list must be a set

    # Add the start node
    open_list.append(start_node)

    # Loop until you find the end
    while len(open_list) > 0:
        if(len(open_list) > 1000):
                break
        #print(len(open_list))
        # Get the current node
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
        # Pop current off open list, add to closed list
        open_list.pop(current_index)
        closed_list.add(current_node)     # <-- change append to add

        # Found the goal
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # Adjacent squares

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range
            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[len(maze)-1]) -1) or node_position[1] < 0:
                continue

            # Make sure walkable terrain
            if maze[int(node_position[0])][int(node_position[1])] != 0:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        for child in children:
            # Child is on the closed list
            if child in closed_list:              # <-- remove inner loop so continue takes you to the end of the outer loop
                continue

            # Create the f, g, and h values
            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            # Child is already in the open list
            for open_node in open_list:
                if child == open_node and child.g > open_node.g:
                    continue

            # Add the child to the open list
            open_list.append(child)


def astarInit(obstacleMap, MAP_SIZE_PIXELS, endx, endy):
    # Maze is map we converted to 0's and 1's
    maze = obstacleMap

    # Robot always starts at center of map
    start = (start_x, start_y)

    # End goal will be passed in
    end = (endx, endy)

    path = astar(maze, start, end)
    return(path)

#Function to convert mapping array from values from 0-255 to 0's representing
# open space and 1's representing an obstacle present.
def convertToObstacleMap(mapping):
    mapping[mapping < 120] = 1
    mapping[mapping >= 120] = 0

    return mapping

def convertPath(start, goal):
    size_per_unit = MAP_SIZE_METERS/MAP_SIZE_PIXELS
    difference_x = int(start[0]) - int(goal[0])
    difference_y = int(start[1]) - int(goal[1])

    distance_move_x = size_per_unit * difference_x * -1
    distance_move_y = size_per_unit * difference_y * -1

    next_pos = [0, 0]

    next_pos[0] = distance_move_x
    next_pos[1] = distance_move_y

    return(next_pos[0], next_pos[1])

# Used to give map some time to update
buffer_counter = 0

while True:
    # Receives raw lidar data
    raw_data = json.loads(socket1.recv())
    raw_data_new = [[i * 1000 for i in raw_data[0:360]]]
    for i in range(360, (len(raw_data))):
        fixed_data_org = [k * 1000 for k in raw_data[i]]
        raw_data_new.append(fixed_data_org)

    raw_data_new = list(raw_data_new)[0]

    # Sends message back to raw data script
    socket1.send_string('Got the raw data: %s ' % raw_data_new)

    # Update SLAM with current Lidar scan
    slam.update(raw_data_new)

    # Get current robot position
    map_x, map_y, map_theta = slam.getpos()
    #print(x, y)

    # Get current map bytes as grayscale
    slam.getmap(mapbytes)

    # Convert byteArray to array containing values 0-255 indicating grayscale colors
    mapping = np.reshape(np.frombuffer(mapbytes, dtype=np.uint8), (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))

    # Convert array containing grayscale colors to 0's for open space and 1's for obstacles
    obstacleMap = convertToObstacleMap(mapping)


    # RUN STEERING ALGORITHM TO PASS THROUGH PATH
    # Display map and robot pose, exiting gracefully if user closes it
    if not viz.display(map_x / 1000., map_y / 1000., map_theta, mapbytes):
        exit(0)

    # Begin running A* algorithm 35, 35,     15, 15    15, 17 (best so far)
    path = astarInit(obstacleMap, MAP_SIZE_PIXELS, 40, 40)


    if len(path) < 2:
        print("Destination reached!")
        break
    else:
        if(path[0][0] > path[1][0]):
           	start_x = start_x - 1
        elif(path[0][0] < path[1][0]):
            start_x = start_x + 1
        else:
        	start_x = start_x

        if(path[0][1] > path[1][1]):
            start_y = start_y - 1
        elif(path[0][1] < path[1][1]):
            start_y = start_y + 1
        else:
        	start_y = start_y

        #print(path)
        
        

    if path is not None:
        converted = convertPath(path[0], path[1])
        #print(converted[0], converted[1])
        moveRobot(converted[0], converted[1])


    print(path)

    #print(map_x, map_y)

    # Path var should contain all points that needed to be visited by Turtlebot
    #print(path)
    #moveRobot(.75, .75)
    #print("exited moveRobot")


