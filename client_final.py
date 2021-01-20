import zmq
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time
import simplejson as json

import pickle
import csv
import os

context = zmq.Context()

# Socket to talk to server
print("Connecting to hello world server…")
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5555")


def callback(msg):
	messages = list(msg.ranges)
	socket.send_string(json.dumps(messages))
	print(messages)
	message = socket.recv()
	print('Got message: %s' % message)

rospy.init_node('laser_data')

sub = rospy.Subscriber('/scan', LaserScan, callback)
move = Twist()

rospy.spin()
