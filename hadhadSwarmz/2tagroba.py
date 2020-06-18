#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
#posewithcovariance bta3t mcl pose
from geometry_msgs.msg import Twist, Point ,PoseWithCovarianceStamped,PoseStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry #to know robot pos
from tf import transformations # gowah conversion quarention to euler
#trajector msg
from visualization_msgs.msg import MarkerArray
import numpy as np
import math

from nav_msgs.srv import GetMap



position_ = PoseWithCovarianceStamped()
yaw_=0
x = [0,0,0]
counter = 0
goal_rob2 = PoseStamped()
goal_rob3 = PoseStamped()
status = 0

#publisher to cmd_vel
pub_rob2 = None
pub_rob3 = None

shape_dist = 1.5

def callbackrob1g(msg):
    global position_
    global yaw_
    global shape_dist
    global goal_rob2
    global goal_rob3
    global counter
    global status
    
    #1st pose is post with covariance
    position_ = msg

    goal_rob2.header.frame_id = position_.header.frame_id
    goal_rob3.header.frame_id = position_.header.frame_id
    
    static_map = rospy.ServiceProxy('static_map',GetMap)
    map = static_map()
    map_info = map.map.info 
    
    quaternion = (
    	position_.pose.pose.orientation.x,
    	position_.pose.pose.orientation.y,
    	position_.pose.pose.orientation.z,
    	position_.pose.pose.orientation.w)
    	
    euler = transformations.euler_from_quaternion(quaternion)
    
    yaw_ = euler[2]
    
    theta = map_info.origin.orientation.z + yaw_ 
    
    if(status == 1):
    	goal_rob2.pose.position.x = position_.pose.pose.position.x - shape_dist*math.sin((math.pi/3 + yaw_))
    	goal_rob2.pose.position.y = position_.pose.pose.position.y + shape_dist*math.cos((math.pi/3 + yaw_))
    	goal_rob2.pose.orientation.z = position_.pose.pose.orientation.z
    	goal_rob2.pose.orientation.w = position_.pose.pose.orientation.w
    
    	goal_rob3.pose.position.x = position_.pose.pose.position.x - shape_dist*math.sin((2*math.pi/3 + yaw_))
    	goal_rob3.pose.position.y = position_.pose.pose.position.y + shape_dist*math.cos((2*math.pi/3 + yaw_))
    	goal_rob3.pose.orientation.z = position_.pose.pose.orientation.z
    	goal_rob3.pose.orientation.w = position_.pose.pose.orientation.w
    	pub_rob2.publish(goal_rob2)
    	pub_rob3.publish(goal_rob3)
    	
    
    if(status == 2):
		goal_rob2.pose.position.x = position_.pose.pose.position.x - shape_dist*math.cos(yaw_)
		goal_rob2.pose.position.y = position_.pose.pose.position.y - shape_dist*math.sin(yaw_)
		goal_rob2.pose.orientation.z = position_.pose.pose.orientation.z
		goal_rob2.pose.orientation.w = position_.pose.pose.orientation.w
		
		goal_rob3.pose.position.x = position_.pose.pose.position.x - 2*shape_dist*math.cos(yaw_)
		goal_rob3.pose.position.y = position_.pose.pose.position.y - 2*shape_dist*math.sin(yaw_)
		goal_rob3.pose.orientation.z = position_.pose.pose.orientation.z
		goal_rob3.pose.orientation.w = position_.pose.pose.orientation.w
		pub_rob2.publish(goal_rob2)
		pub_rob3.publish(goal_rob3)
		counter+=1
   
def bab(msg):
	
	global x
	
	x[1] = x[0]
	x[0] = msg.data
	
	eb3t()
		

		
		
	
def eb3t():
	global status
	global counter
	global x
	print(counter)
	if((x[0] == x[1] == True) or (counter < 15 and counter > 0)):
		#print('bab')
		status = 2
	elif(counter >= 15):
		counter = 0
		status = 1
	else :
		#print(' ')
		status = 1
		
		

def main():
    global pub_rob2
    global pub_rob3
    global rate
    rospy.init_node('hadhad_swarmz')

    pub_rob2 = rospy.Publisher('/rob2/move_base_simple/goal',PoseStamped,queue_size=1000)
    pub_rob3 = rospy.Publisher('/rob3/move_base_simple/goal',PoseStamped,queue_size=1000)

    sub_rob1g = rospy.Subscriber('/rob1/amcl_pose',PoseWithCovarianceStamped,callbackrob1g)
    sub_bab = rospy.Subscriber('/rob1/bab',Bool,bab)

    rate = rospy.Rate(20)

    rospy.spin()
  
if __name__ == "__main__":
    main()

