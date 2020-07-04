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

#This node controls the formation of the robots while they move. 

#gets arguments given by the launch file and but it in Robot_ID variable
args = rospy.myargv(argv=sys.argv)
Robot_ID = int(args[1]) #stores in it the ID of the robot
 
position_ = PoseWithCovarianceStamped() #global variable to store in it data with type of PoseWithCovarianceStamped
yaw_=0 #stores in it the orientation of the robot in radians
x = False #stores in it if the leader went through a narrow path
counter = 0 #counts some time before returning from line formation to a traingle formation
goal = PoseStamped() #variable stores in it the next goal of a follower robot
goal_lead = PoseStamped() #stores in it the leader goal
status = 0 #status = 1 if the formation is a triangle and status = 2 if the formation is a line

#publisher to cmd_vel
pub_goal = None

sub_rob1 = [None] #Subscriber to the amcl position topic of the 1st robot
sub_rob2 = [None] #Subscriber to the amcl position topic of the 2nd robot
sub_rob3 = [None] #Subscriber to the amcl position topic of the 3rd robot
subs = [sub_rob1 , sub_rob2 , sub_rob3] #list containing all the robots' subscribers

sub_bab1 = [None] #Subscriber to the Narrow path topic of the 1st robot
sub_bab2 = [None] #Subscriber to the Narrow path topic of the 2nd robot
sub_bab3 = [None] #Subscriber to the Narrow path topic of the 3rd robot
subs_bab = [sub_bab1 , sub_bab2 , sub_bab3] #list containing all the robots' subscribers

lead = None #variable containing the robot ID of the leader
fol1 = None #variable containing the robot ID of the 1st follower
fol2 = None #variable containing the robot ID of the 2nd follower

shape_dist = 1.5 #stores in it the length of the edge of the triangle to be made

#call back of the leader amcl position to calculate on the leader position the position of the 1st or 2nd follower
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
    
    position_ = msg #stores in it the position of the leader robot
    
    goal.header.frame_id = position_.header.frame_id #put the frame id in the goal variable

    #gets the map info
    static_map = rospy.ServiceProxy('static_map',GetMap)
    map = static_map()
    map_info = map.map.info
        
    #put the orientation of the leader robot in the quaternion variable
    quaternion = (
    	position_.pose.pose.orientation.x,
    	position_.pose.pose.orientation.y,
    	position_.pose.pose.orientation.z,
    	position_.pose.pose.orientation.w)
    
    #transform the orientation of the leader robot from quaternion to euler	
    euler = transformations.euler_from_quaternion(quaternion)
    
    #stores the orientation of the 1st robot in the form of radians
    yaw_ = euler[2]
    
    theta = map_info.origin.orientation.z + yaw_ #idk what is it Hady kan katbha w 3omry mashoftaha mosta5dma
    
    #if the robot ID is follower1 calculate the required position of follower1 and publish it
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
    		counter+=1 #increament the counter until reaching 15 to return to triangle formation
    
    #else if the robot ID is follower2 calculate the required position of follower2 and publish it
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
			counter+=1 #increament the counter until reaching 15 to return to triangle formation
			   
#call back function to the narrow path topic of leader and call the eb3t function
def bab(msg):
	
	global x
	x = msg.data
	eb3t()

#call back function to the leader topic
def lead_get(msg):
	global subs
	global subs_bab
	global lead
	
	#unsubscribe the amcl position and the narrow path topics of the non leader robots and stores the leader ID in lead 
	for i in range(0,3):
		if(i != msg.data):
			subs[i].unregister()
			subs_bab[i].unregister()
		elif(i == msg.data):
			lead = msg.data
			
#call back function to the follower1 topic
def fol1_get(msg):
	global fol1
	fol1 = msg.data

#call back function to the follower2 topic
def fol2_get(msg):
	global fol2
	fol2 = msg.data	

#call back function to goal put in RVIZ topic
def goal_get(msg):
	global goal_lead
	global lead
	global Robot_ID
	global pub_goal
	
	goal_lead = msg
	
	#sends the goal to the leader robot
	if(lead == Robot_ID):
		pub_goal.publish(goal_lead)
		
#function that controls if the formation is triangle or line
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

