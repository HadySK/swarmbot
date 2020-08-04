#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
#posewithcovariance bta3t mcl pose
from geometry_msgs.msg import Twist, Point ,PoseWithCovarianceStamped,PoseStamped,Pose
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry #to know robot pos
from tf import transformations # gowah conversion quarention to euler
#trajector msg
from visualization_msgs.msg import MarkerArray
import numpy as np
import math

#This node takes the robots seen by every robot and determine which robot to be leader and which to be fol1 and fol2

sub_rob1 = None #Subscriber to the robots_seen topic of the 1st robot
sub_rob2 = None #Subscriber to the robots_seen topic of the 2nd robot
sub_rob3 = None #Subscriber to the robots_seen topic of the 3rf robot
sub_rob_pos = None #Subscriber to the position topic of the 1st robot
sub_rob2_pos = None #Subscriber to the position topic of the 2nd robot
sub_rob3_pos = None #Subscriber to the position topic of the 3rd robot
pub_led  = None #Publisher to publish the robot ID of the leader
pub_fol1 = None #Publisher to publish the robot ID of follower 1
pub_fol2 = None	#Publisher to publish the robot ID of follower 2
pub_goal = None
goalpose = PoseStamped()

Dist_rob1 = None
Dist_rob2 = None	
Dist_rob3 = None
Distance = [Dist_rob1 , Dist_rob2 , Dist_rob3]
Near_ID = 0 #Nearest Robot to Goal ID

Pose_rob = Pose() #Pose variable to store in it the position of 1st robot
Pose_rob2 = Pose() #Pose variable to store in it the position of 2nd robot
Pose_rob3 = Pose() #Pose variable to store in it the position of 3rd robot

rob1_seen = [0] #global variable to store in it the number of robots seen by the 1st robot
rob2_seen = [0] #global variable to store in it the number of robots seen by the 2nd robot
rob3_seen = [0] #global variable to store in it the number of robots seen by the 3rd robot
robs_seen = [rob1_seen , rob2_seen , rob3_seen] #list to store the robots seen by all robots

leader = 0 #stores in it the robot ID of the leader
follower1 = 0 #stores in it the robot ID of follower1
follower2 = 0 #stores in it the robot ID of follower2
status = 1 #controls when to combute which robot is leader and which is fol1 and fol2
shape_dist = 1.5 #stores in it the length of the edge of the triangle to be made

#call back function to sub_rob1 to store the number of robots seen by 1st robot
def callbackrob1(msg):
    global rob1_seen
    rob1_seen[0] = msg.data

#call back function to sub_rob2 to store the number of robots seen by 2nd robot
def callbackrob2(msg):
    global rob2_seen
    rob2_seen[0] = msg.data

#call back function to sub_rob3 to store the number of robots seen by 3rd robot
def callbackrob3(msg):
    global rob3_seen
    rob3_seen[0] = msg.data

#call back function to sub_rob_pos to store the position of the 1st robot
def callbackrob4(msg):
    global Pose_rob
    Pose_rob = msg

#call back function to sub_rob2_pos to store the position of the 2nd robot
def callbackrob5(msg):
    global Pose_rob2
    Pose_rob2 = msg

#call back function to sub_rob3_pos to store the position of the 3rd robot
def callbackrob6(msg):
    global Pose_rob3
    Pose_rob3 = msg

#call back function to calculate distance between each robot and the goal
def distance(msg):
	global Pose_rob
	global Pose_rob2
	global Pose_rob3
	global Near_ID
	global Distance
	global goalpose
	goalpose = msg 
	Dist_rob1 = math.sqrt(math.pow(goalpose.pose.position.x - Pose_rob.position.x , 2) + math.pow(goalpose.pose.position.y- 		Pose_rob.position.y, 2))
	Dist_rob2 = math.sqrt(math.pow(goalpose.pose.position.x - Pose_rob2.position.x , 2) + math.pow(goalpose.pose.position.y- 		Pose_rob2.position.y, 2))
	Dist_rob3 = math.sqrt(math.pow(goalpose.pose.position.x - Pose_rob3.position.x , 2) + math.pow(goalpose.pose.position.y- 		Pose_rob3.position.y, 2))
	
	Distance = [Dist_rob1 , Dist_rob2 , Dist_rob3]
	for i in range(0,3):
		if(Distance[i] < Distance[Near_ID]):
			Near_ID = i
	print(Near_ID)
	if(status == 1):
		control()	

#call this function when status = 1 to combute the status of the robot
def control():
	global robs_seen
	global Pose_rob
	global Pose_rob2
	global Pose_rob3
	global leader
	global follower1
	global follower2
	global status
	global Near_ID
	global goalpose
	global pub_goal
	
	pose = [Pose_rob , Pose_rob2 , Pose_rob3] #list containing all the robots positions
	yaw_ = [0,0,0] #list to store in it all the robots position
	distance = 1000 #used to see which follower is closer to the position to position of follower 1 to assign it
	print(robs_seen)
	
	#put the orientation of the 1st robot in the quaternion variable
	quaternion = (
		Pose_rob.orientation.x,
		Pose_rob.orientation.y,
		Pose_rob.orientation.z,
		Pose_rob.orientation.w)
		
	#transform the orientation of 1st robot from quaternion to euler
	euler = transformations.euler_from_quaternion(quaternion)
	
	#stores the orientation of the 1st robot in the form of radians
	yaw_[0] = euler[2]
	
	#put the orientation of the 2nd robot in the quaternion variable
	quaternion = (
		Pose_rob2.orientation.x,
		Pose_rob2.orientation.y,
		Pose_rob2.orientation.z,
		Pose_rob2.orientation.w)
	
	#transform the orientation of 2nd robot from quaternion to euler
	euler = transformations.euler_from_quaternion(quaternion)
	
	#stores the orientation of the 2nd robot in the form of radians
	yaw_[1] = euler[2]
	
	#put the orientation of the 3rd robot in the quaternion variable
	quaternion = (
		Pose_rob3.orientation.x,
		Pose_rob3.orientation.y,
		Pose_rob3.orientation.z,
		Pose_rob3.orientation.w)
	
	#transform the orientation of 3rd robot from quaternion to euler
	euler = transformations.euler_from_quaternion(quaternion)
	
	#stores the orientation of the 3rd robot in the form of radians
	yaw_[2] = euler[2]
	
	#sees the robot with highest number of robots seen to be leader
	for i in range(1,3):
		if(robs_seen[i] > robs_seen[leader]):
			leader = i
		if(robs_seen[i] == robs_seen[leader]):
			if(i == Near_ID):
				leader = i;
			elif(Distance[i] < Distance[leader]):
				leader = i;
			
	#calculates the x and y positions of follower 1
	followerx = pose[leader].position.x - shape_dist*math.sin((math.pi/3 + yaw_[leader]))
	followery = pose[leader].position.y + shape_dist*math.cos((math.pi/3 + yaw_[leader]))
	
	#sees which robots that are not leader that is closest to follower1 position to be assigned as follower 1	
	for i in range(0,3):
		if(i != leader):
			if(math.sqrt(math.pow(followerx - pose[i].position.x , 2) + math.pow(followery - pose[i].position.y , 2)) < distance):
				distance = math.sqrt(math.pow(followerx - pose[i].position.x , 2) + math.pow(followery - pose[i].position.y , 2))
				follower1 = i
	
	#assign the last robot as follower 2
	for i in range(0,3):
		if(i != leader and i != follower1):
			follower2 = i
	
	#change status to 0 so it doesn't keeps calculating the robots status while they are moving
	status = 0
	
def main():
	global sub_rob1
	global sub_rob2
	global sub_rob3
	global sub_rob_pos
	global pub_led
	global pub_fol1
	global pub_fol2
	global status
	global leader
	global follower1
	global follower2
	global pub_goal
	global goalpose
    	
	rospy.init_node('Leader')
	
	sub_rob1 = rospy.Subscriber('/rob1/robots_seen',Int8,callbackrob1)
	sub_rob2 = rospy.Subscriber('/rob2/robots_seen',Int8,callbackrob2)
	sub_rob3 = rospy.Subscriber('/rob3/robots_seen',Int8,callbackrob3)
	sub_rob_pos = rospy.Subscriber('/rob1/pose',Pose,callbackrob4)
	sub_rob2_pos = rospy.Subscriber('/rob2/pose',Pose,callbackrob5)
	sub_rob3_pos = rospy.Subscriber('/rob3/pose',Pose,callbackrob6)
	
	pub_led = rospy.Publisher('/leader',Int8,queue_size=1)
	pub_fol1 = rospy.Publisher('/follower1',Int8,queue_size=1)
	pub_fol2 = rospy.Publisher('/follower2',Int8,queue_size=1)
	
	sub_goal = rospy.Subscriber('/move_base_simple/goal',PoseStamped,distance)
	pub_goal = rospy.Publisher('/goalx',PoseStamped,queue_size=1)
	
	rate = rospy.Rate(20)
	while not rospy.is_shutdown() :
		rate.sleep()#delay to wait until poses of the robots are published else the calculations may be wrong
		rate.sleep()
		rate.sleep()
		rate.sleep()	
		#keeps publishing the robots' ID on the topic of each status
		pub_led.publish(leader)
		pub_fol1.publish(follower1)
		pub_fol2.publish(follower2)
		if(goalpose != None):
			pub_goal.publish(goalpose)
	else:
		rospy.logerr('Unknown state!')
	
	rospy.spin()
  
if __name__ == "__main__":
    main()

