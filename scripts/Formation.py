#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point ,PoseWithCovarianceStamped,PoseStamped , Pose
from std_msgs.msg import Bool , Int8
from actionlib_msgs.msg import GoalStatusArray
from nav_msgs.msg import Odometry
from tf import transformations
from visualization_msgs.msg import MarkerArray
import numpy as np
import math
import sys
from nav_msgs.srv import GetMap

#This node controls the formation of the robots while they move.

SHAPE_DIST = 1.5 #stores in it the length of the edge of the triangle to be made

#values that is used in 'Formation_Shape' variable
TRIANGLE = 1
LINE = 2
DIAGONAL = 3
INV_DIAGONAL =4

#values that is used in 'NarrowPath_Status' variable
NO_NARROWPATH = 0
LEADER_PASSED = 1
FOLLOWER_PASSED = 2

#values that is used in 'goal_lead_status' variable
NotReady_toSend = 0
Got_LeaderID = 1
Ready_toSend = 3

#values that is used in 'robot_status' variable
No_Goal = 0
Reached_prevGoal = 3

#gets arguments given by the launch file and but it in Robot_ID variable
args = rospy.myargv(argv=sys.argv)
Robot_ID = int(args[1]) #stores in it the ID of the robot


#global variables used in this node
position_ = PoseWithCovarianceStamped() #global variable to store in it data with type of PoseWithCovarianceStamped
yaw_=0 #stores in it the orientation of the robot in radians
is_NarrowPath = False #stores in it if the leader went through a narrow path
NarrowPath_Status = NO_NARROWPATH #status for coordinating the return from line formation to a traingle formation
goal = PoseStamped() #variable stores in it the next goal of a follower robot
goal_lead = PoseStamped() #stores in it the leader goal
Formation_Shape = TRIANGLE #stores in it the formation shape to be made
prev_Shape = TRIANGLE #stores in it the previous shape that was made before making a line to go through a narrow path
goal_lead_status = NotReady_toSend #used to control to send a new goal to the leader when it didn't get a goal before or when it reached its prevous goal
robot_status = No_Goal #stores in it the goal status of the robot
lead = -1 #variable containing the robot ID of the leader
fol1 = -1 #variable containing the robot ID of the 1st follower
fol2 = -1 #variable containing the robot ID of the 2nd follower

#global publishers and subscribers
pub_goal = None #publisher to cmd_vel of the robot

sub_rob1 = [None] #Subscriber to the amcl position topic of the 1st robot
sub_rob2 = [None] #Subscriber to the amcl position topic of the 2nd robot
sub_rob3 = [None] #Subscriber to the amcl position topic of the 3rd robot
subs = [sub_rob1 , sub_rob2 , sub_rob3] #list containing all the robots' subscribers

sub_isNarrow1 = [None] #Subscriber to the Narrow path topic of the 1st robot
sub_isNarrow2 = [None] #Subscriber to the Narrow path topic of the 2nd robot
sub_isNarrow3 = [None] #Subscriber to the Narrow path topic of the 3rd robot
subs_isNarrow = [sub_isNarrow1 , sub_isNarrow2 , sub_isNarrow3] #list containing all the robots' subscribers

sub_goal = None #Subscriber to the leader goal

Narrow1 = [None] #Subscriber to the Narrow path topic of the 1st robot
Narrow2 = [None] #Subscriber to the Narrow path topic of the 2nd robot
Narrow3 = [None] #Subscriber to the Narrow path topic of the 3rd robot
Narrow  = [Narrow1 , Narrow2 , Narrow3] #list containing all the robots' subscribers

sub_s1 = [None] #Subscriber to the amcl position topic of the 1st robot
sub_s2 = [None] #Subscriber to the amcl position topic of the 2nd robot
sub_s3 = [None] #Subscriber to the amcl position topic of the 3rd robot
subs_s = [sub_s1 , sub_s2 , sub_s3] #list containing all the robots' subscribers

Door1 = False    #1st Robot Door Flag
Door2 = False	 #2nd Robot Door Flag
Door3 = False	 #3nd Robot Door Flag
Door  = [Door1 , Door2 , Door3] #list containing all the robots' subscribers


#call back of the leader amcl position to calculate on the leader position, the position of the 1st or 2nd follower
def callbackrob1g(msg):
    global position_, yaw_, goal, fol1, fol2, pub_goal, Formation_Shape, Robot_ID

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

    theta = map_info.origin.orientation.z + yaw_ #store the orientation in degree in it
    #if the robot ID is follower1 calculate the required position of follower1 and publish it
    if(fol1 == Robot_ID):

        if(Formation_Shape == TRIANGLE):
    		goal.pose.position.x = position_.pose.pose.position.x - SHAPE_DIST*math.sin((math.pi/3 + yaw_))
    		goal.pose.position.y = position_.pose.pose.position.y + SHAPE_DIST*math.cos((math.pi/3 + yaw_))
    		goal.pose.orientation.z = position_.pose.pose.orientation.z
    		goal.pose.orientation.w = position_.pose.pose.orientation.w
    		pub_goal.publish(goal)

        elif(Formation_Shape == LINE):
    		goal.pose.position.x = position_.pose.pose.position.x - SHAPE_DIST*math.cos(yaw_)
    		goal.pose.position.y = position_.pose.pose.position.y - SHAPE_DIST*math.sin(yaw_)
    		goal.pose.orientation.z = position_.pose.pose.orientation.z
    		goal.pose.orientation.w = position_.pose.pose.orientation.w
    		pub_goal.publish(goal)

    	elif(Formation_Shape == DIAGONAL):
    		goal.pose.position.x = position_.pose.pose.position.x - SHAPE_DIST*math.sin((math.pi/3 + yaw_))
    		goal.pose.position.y = position_.pose.pose.position.y + SHAPE_DIST*math.cos((math.pi/3 + yaw_))
    		goal.pose.orientation.z = position_.pose.pose.orientation.z
    		goal.pose.orientation.w = position_.pose.pose.orientation.w
    		pub_goal.publish(goal)

    	elif(Formation_Shape == INV_DIAGONAL):
    		goal.pose.position.x = position_.pose.pose.position.x - SHAPE_DIST*math.sin((2*math.pi/3 + yaw_))
    		goal.pose.position.y = position_.pose.pose.position.y + SHAPE_DIST*math.cos((2*math.pi/3 + yaw_))
    		goal.pose.orientation.z = position_.pose.pose.orientation.z
    		goal.pose.orientation.w = position_.pose.pose.orientation.w
    		pub_goal.publish(goal)

    #else if the robot ID is follower2 calculate the required position of follower2 and publish it
    elif(fol2 == Robot_ID):

        if(Formation_Shape == TRIANGLE):
			goal.pose.position.x = position_.pose.pose.position.x - SHAPE_DIST*math.sin((2*math.pi/3 + yaw_))
			goal.pose.position.y = position_.pose.pose.position.y + SHAPE_DIST*math.cos((2*math.pi/3 + yaw_))
			goal.pose.orientation.z = position_.pose.pose.orientation.z
			goal.pose.orientation.w = position_.pose.pose.orientation.w
			pub_goal.publish(goal)

        elif(Formation_Shape == LINE):
			goal.pose.position.x = position_.pose.pose.position.x - 2*SHAPE_DIST*math.cos(yaw_)
			goal.pose.position.y = position_.pose.pose.position.y - 2*SHAPE_DIST*math.sin(yaw_)
			goal.pose.orientation.z = position_.pose.pose.orientation.z
			goal.pose.orientation.w = position_.pose.pose.orientation.w
			pub_goal.publish(goal)

        elif(Formation_Shape == DIAGONAL):
			goal.pose.position.x = position_.pose.pose.position.x - 2*SHAPE_DIST*math.sin((math.pi/3 + yaw_))
			goal.pose.position.y = position_.pose.pose.position.y + 2*SHAPE_DIST*math.cos((math.pi/3 + yaw_))
			goal.pose.orientation.z = position_.pose.pose.orientation.z
			goal.pose.orientation.w = position_.pose.pose.orientation.w
			pub_goal.publish(goal)

        elif(Formation_Shape == INV_DIAGONAL):
			goal.pose.position.x = position_.pose.pose.position.x - 2*SHAPE_DIST*math.sin((2*math.pi/3 + yaw_))
			goal.pose.position.y = position_.pose.pose.position.y + 2*SHAPE_DIST*math.cos((2*math.pi/3 + yaw_))
			goal.pose.orientation.z = position_.pose.pose.orientation.z
			goal.pose.orientation.w = position_.pose.pose.orientation.w
			pub_goal.publish(goal)

#call back function to the narrow path topic of leader and call the Control_NarrowPath function
def isNarrow(msg):
	global is_NarrowPath
	is_NarrowPath = msg.data
	Control_NarrowPath()

def isNarrow1(msg):
	global Door
	Door[0] = msg.data
	Control_NarrowPath()

def isNarrow2(msg):
	global Door
	Door[1] = msg.data
	Control_NarrowPath()

def isNarrow3(msg):
	global Door
	Door[2] = msg.data
	Control_NarrowPath()

#call back function to the leader topic
def lead_get(msg):
	global subs, subs_isNarrow, lead, goal_lead_status
	#unsubscribe the amcl position and the narrow path topics of the non leader robots and stores the leader ID in lead
	for i in range(0,3):
		if(i != msg.data):
			subs[i].unregister()
			subs_isNarrow[i].unregister()
			subs_s[i].unregister()
		elif(i == msg.data):
			lead = msg.data
			goal_lead_status = Got_LeaderID

#call back function that decides if the leader robot can get a new goal at this moment or not
def robot_status(msg):
	global robot_status, goal_lead_status
	robot_status = msg.status_list
	if((len(robot_status) == No_Goal or robot_status[0].status == Reached_prevGoal) and goal_lead_status == Got_LeaderID):
		goal_lead_status = Ready_toSend

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
	global goal_lead, lead, Robot_ID, pub_goal, goal_lead_status

	goal_lead = msg
	#sends the goal to the leader robot
	if(lead == Robot_ID and goal_lead_status == Ready_toSend):
		pub_goal.publish(goal_lead)
		goal_lead_status = NotReady_toSend


#function that controls the passage through a narrow path, by temporarily changing the formation to a line
def Control_NarrowPath():
	global Formation_Shape, NarrowPath_Status, prev_Shape

	if((Door[lead] == True )and (NarrowPath_Status == NO_NARROWPATH)):
		NarrowPath_Status = LEADER_PASSED
		prev_Shape = Formation_Shape
		Formation_Shape = LINE
	if((Door[fol1] == True) and (NarrowPath_Status == LEADER_PASSED)):
		NarrowPath_Status = FOLLOWER_PASSED
	if((Door[fol2] == True) and (NarrowPath_Status == LEADER_PASSED)):
		NarrowPath_Status = FOLLOWER_PASSED
	if((Door[lead] == False) and (NarrowPath_Status == FOLLOWER_PASSED)):
		NarrowPath_Status = NO_NARROWPATH
		Formation_Shape = prev_Shape

#call back function to get the formation shape the user send on the ROS topic
def shape_get(msg):
	global Formation_Shape
	Formation_Shape = msg.data

def main():
	global pub_goal, subs, subs_isNarrow, sub_goal, subs_s

	rospy.init_node('Formation')

	pub_goal = rospy.Publisher('/goal',PoseStamped,queue_size=1000)
	subs[0] = rospy.Subscriber('/rob1/amcl_pose',PoseWithCovarianceStamped,callbackrob1g)
	subs[1] = rospy.Subscriber('/rob2/amcl_pose',PoseWithCovarianceStamped,callbackrob1g)
	subs[2] = rospy.Subscriber('/rob3/amcl_pose',PoseWithCovarianceStamped,callbackrob1g)

	subs_isNarrow[0] = rospy.Subscriber('/rob1/isNarrow',Bool,isNarrow)
	subs_isNarrow[1] = rospy.Subscriber('/rob2/isNarrow',Bool,isNarrow)
	subs_isNarrow[2] = rospy.Subscriber('/rob3/isNarrow',Bool,isNarrow)

	subs_s[0] = rospy.Subscriber('/rob1/move_base/status',GoalStatusArray,robot_status)
	subs_s[1] = rospy.Subscriber('/rob2/move_base/status',GoalStatusArray,robot_status)
	subs_s[2] = rospy.Subscriber('/rob3/move_base/status',GoalStatusArray,robot_status)

	sub_goal = rospy.Subscriber('/goalx',PoseStamped,goal_get)
	sub_shape = rospy.Subscriber('/shape',Int8,shape_get)

	sub_lead = rospy.Subscriber('/leader',Int8,lead_get)
	sub_fol1_get = rospy.Subscriber('/follower1',Int8,fol1_get)
	sub_fol2_get = rospy.Subscriber('/follower2',Int8,fol2_get)

	Narrow[0] = rospy.Subscriber('/rob1/isNarrow',Bool,isNarrow1)
	Narrow[1] = rospy.Subscriber('/rob2/isNarrow',Bool,isNarrow2)
	Narrow[2] = rospy.Subscriber('/rob3/isNarrow',Bool,isNarrow3)

	rate = rospy.Rate(20)
	rate.sleep()
	rospy.spin()

if __name__ == "__main__":
    main()
