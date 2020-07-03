#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
#posewithcovariance bta3t mcl pose
from geometry_msgs.msg import Twist, Point ,PoseWithCovarianceStamped,PoseStamped , Pose
from std_msgs.msg import Bool , Int8
from nav_msgs.msg import Odometry #to know robot pos
from tf import transformations # gowah conversion quarention to euler
#trajector msg
from visualization_msgs.msg import MarkerArray
import numpy as np
import math
import sys

from nav_msgs.srv import GetMap

args = rospy.myargv(argv=sys.argv)
Robot_ID = int(args[1])
position_ = PoseWithCovarianceStamped()
yaw_=0
x = False
counter = 0
goal = PoseStamped()
goal_lead = PoseStamped()
status = 0

#publisher to cmd_vel
pub_goal = None

sub_rob1 = [None]
sub_rob2 = [None]
sub_rob3 = [None]
subs = [sub_rob1 , sub_rob2 , sub_rob3]

sub_bab1 = [None]
sub_bab2 = [None]
sub_bab3 = [None]
subs_bab = [sub_bab1 , sub_bab2 , sub_bab3]

lead = None
fol1 = None
fol2 = None

shape_dist = 1.5

def callbackrob1g(msg):
    global position_
    global yaw_
    global shape_dist
    global goal
    global fol1
    global fol2
    global pub_goal
    global counter
    global status
    global goal_lead
    global Robot_ID
    global lead
    global subs
    global subs_bab
    
    #1st pose is post with covariance
    position_ = msg
    
    goal.header.frame_id = position_.header.frame_id

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
    
    if(fol1 == Robot_ID):
    	if(status == 1):
    		goal.pose.position.x = position_.pose.pose.position.x - shape_dist*math.sin((math.pi/3 + yaw_))
    		goal.pose.position.y = position_.pose.pose.position.y + shape_dist*math.cos((math.pi/3 + yaw_))
    		goal.pose.orientation.z = position_.pose.pose.orientation.z
    		goal.pose.orientation.w = position_.pose.pose.orientation.w
    		pub_goal.publish(goal)
    	elif(status == 2):
    		goal.pose.position.x = position_.pose.pose.position.x - shape_dist*math.cos(yaw_)
    		goal.pose.position.y = position_.pose.pose.position.y - shape_dist*math.sin(yaw_)
    		goal.pose.orientation.z = position_.pose.pose.orientation.z
    		goal.pose.orientation.w = position_.pose.pose.orientation.w
    		pub_goal.publish(goal)
    		counter+=1
    
    elif(fol2 == Robot_ID):
		if(status == 1):
			goal.pose.position.x = position_.pose.pose.position.x - shape_dist*math.sin((2*math.pi/3 + yaw_))
			goal.pose.position.y = position_.pose.pose.position.y + shape_dist*math.cos((2*math.pi/3 + yaw_))
			goal.pose.orientation.z = position_.pose.pose.orientation.z
			goal.pose.orientation.w = position_.pose.pose.orientation.w
			pub_goal.publish(goal)
		elif(status == 2):
			goal.pose.position.x = position_.pose.pose.position.x - 2*shape_dist*math.cos(yaw_)
			goal.pose.position.y = position_.pose.pose.position.y - 2*shape_dist*math.sin(yaw_)
			goal.pose.orientation.z = position_.pose.pose.orientation.z
			goal.pose.orientation.w = position_.pose.pose.orientation.w
			pub_goal.publish(goal)
			counter+=1
			   
def bab(msg):
	
	global x
	x = msg.data
	eb3t()
		
def lead_get(msg):
	global subs
	global subs_bab
	global lead
	for i in range(0,3):
		if(i != msg.data):
			subs[i].unregister()
			subs_bab[i].unregister()
		elif(i == msg.data):
			lead = msg.data
			
def fol1_get(msg):
	global fol1
	fol1 = msg.data

def fol2_get(msg):
	global fol2
	fol2 = msg.data	

def goal_get(msg):
	global goal_lead
	global lead
	global Robot_ID
	global pub_goal
	
	goal_lead = msg
	if(lead == Robot_ID):
		pub_goal.publish(goal_lead)
		
def eb3t():
	global status
	global counter
	global x
	#print(counter)
	if((x == True) or (counter < 15 and counter > 0)):
		#print('bab')
		status = 2
	elif(counter >= 15):
		counter = 0
		status = 1
	else :
		#print(' ')
		status = 1
		
		

def main():
	global pub_goal
	global subs
	global subs_bab
	global rate
	
	rospy.init_node('hadhad_swarmz')
	
	pub_goal = rospy.Publisher('/goal',PoseStamped,queue_size=1000)
	subs[0] = rospy.Subscriber('/rob1/amcl_pose',PoseWithCovarianceStamped,callbackrob1g)
	subs[1] = rospy.Subscriber('/rob2/amcl_pose',PoseWithCovarianceStamped,callbackrob1g)
	subs[2] = rospy.Subscriber('/rob3/amcl_pose',PoseWithCovarianceStamped,callbackrob1g)
	
	subs_bab[0] = rospy.Subscriber('/rob1/bab',Bool,bab)
	subs_bab[1] = rospy.Subscriber('/rob2/bab',Bool,bab)
	subs_bab[2] = rospy.Subscriber('/rob3/bab',Bool,bab)
	
	sub_goal = rospy.Subscriber('/move_base_simple/goal',PoseStamped,goal_get)
	
	sub_lead = rospy.Subscriber('/leader',Int8,lead_get)
	sub_fol1_get = rospy.Subscriber('/follower1',Int8,fol1_get)
	sub_fol2_get = rospy.Subscriber('/follower2',Int8,fol2_get)
	
	rate = rospy.Rate(20)
	rate.sleep()
	rospy.spin()
  
if __name__ == "__main__":
    main()

