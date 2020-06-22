#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
#posewithcovariance bta3t mcl pose
from geometry_msgs.msg import Twist, Point ,PoseWithCovarianceStamped,PoseStamped
from std_msgs.msg import Bool, Int8
from nav_msgs.msg import Odometry #to know robot pos
from tf import transformations # gowah conversion quarention to euler
#trajector msg
from visualization_msgs.msg import MarkerArray
import numpy as np
import math

from nav_msgs.srv import GetMap


sub_rob1 = [None]
sub_rob2 = [None]
sub_rob3 = [None]
subs = [sub_rob1 , sub_rob2 , sub_rob3]

position_ = []
y = []
l = 0
l2 = 0
pub_bab = rospy.Publisher('/rob1/bab',Bool,queue_size=1000)

def callbackrob1g(msg):
    global position_
    global y
    global l
    global l2
    #1st pose is post with covariance
    position_ = msg.ranges
    for i in range(108):
    	y = np.append(y,position_[i])
    for i in range(115,250):
    	y = np.append(y,position_[i])
    l = np.amin(y)
    y = []
    for i in range(108):
    	y = np.append(y,position_[719-i])
    for i in range(115,250):
    	y = np.append(y,position_[719-i])
    l2 = np.amin(y)
    eb3t()
    #print(position_[560])
   
def lead_get(msg):
	global subs
	for i in range(0,3):
		if(i != msg.data):
			subs[i].unregister()	
	
def main():

    global subs
    
    rospy.init_node('hadhad_laser')

    subs[0] = rospy.Subscriber('/rob1/scan',LaserScan,callbackrob1g)
    subs[1] = rospy.Subscriber('/rob2/scan',LaserScan,callbackrob1g)
    subs[2] = rospy.Subscriber('/rob3/scan',LaserScan,callbackrob1g)
    sub_leader = rospy.Subscriber('/leader',Int8,lead_get)

    rate = rospy.Rate(20)

    rospy.spin()
def eb3t():
	
	global l,l2
	x = l+l2
	if( x < 1.0 and x > 0.7):
		pub_bab.publish(True)
	else :
		pub_bab.publish(False)
  
if __name__ == "__main__":
    main()

