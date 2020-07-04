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

#This node senses if the leader robot moved through a narrow path like a door

sub_rob = None #Subscriber to the Laser scan topic of the robot

position_ = [] #stores in it the laser scan message
y = [] #stores in it the left 180 degrees of the laser scan then the right 180 degrees 
l = 0 #stores the minimum reading of the left 180 degrees
l2 = 0 #stores the minimum reading of the right 180 degrees

pub_bab = rospy.Publisher('/bab',Bool,queue_size=1000) #Publisher to publish on a topic if there is a narrow path

#call back function to laser scan to compute if there is a narrow path
def callbackrob1g(msg):
    global position_
    global y
    global l
    global l2
    
    #stores the laser scan message in the position_ variable
    position_ = msg.ranges
    
    #check in the first 180 degrees if there is a narrow path but didn't put the 3 degrees from 108 to 114
    #which are 54 to 57 degrees as the wheels are sensed by the laser so there readings are not obstcales
    for i in range(108):
    	y = np.append(y,position_[i])
    for i in range(115,250):
    	y = np.append(y,position_[i])
    
    #gets the minimum value of the left 180 degrees
    l = np.amin(y)
    
    #clear the left 180 degrees to start putting the right 180 degrees in the list
    y = []
    for i in range(108):
    	y = np.append(y,position_[719-i])
    for i in range(115,250):
    	y = np.append(y,position_[719-i])
    
    #gets the minimum value of the right 180 degrees
    l2 = np.amin(y)
    
    #calls the function
    eb3t()

#This functions determine if the robot is going through a narrow path or not
def eb3t():
	
	global l,l2
	x = l+l2
	if( x < 1.0 and x > 0.7):
		pub_bab.publish(True)
	else :
		pub_bab.publish(False)
   	
	
def main():

    global subs
    
    rospy.init_node('rob1_laser')

    sub_rob = rospy.Subscriber('/scan',LaserScan,callbackrob1g)

    rate = rospy.Rate(20)

    rospy.spin()
  
if __name__ == "__main__":
    main()

