#!/usr/bin/env python

import rospy
#posewithcovariance bta3t mcl pose
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8
from tf import transformations # gowah conversion quarention to euler
import numpy as np
import math

#This nodes is a simulation of peer detection instead of Zedd cam

CAM_FOCAL_LENGTH = 3.33 #The focal length of the camera 
CAM_ANGLE_OF_VIEW = 2 * math.pi / 3 #The angle of view of the camera

#publisher to cmd_vel
sub_rob = None #Subscriber to the position topic of the 1st robot
sub_rob2 = None #Subscriber to the position topic of the 2nd robot
sub_rob3 = None #Subscriber to the position topic of the 3rd robot
pub_rob = None  #Publisher to publish the robots seen by the robot
Pose_rob = Pose() #Pose variable to store in it the position of 1st robot
Pose_rob2 = Pose() #Pose variable to store in it the position of 2nd robot
Pose_rob3 = Pose() #Pose variable to store in it the position of 3rd robot

rob_seen = 0 #global variable to store in it the number of robots seen
status = 1 #global variable used to control when to combute the robots seen by the robot

#call back function to sub_rob to store the position of 1st robot
def callbackrob1(msg):
    global Pose_rob
    Pose_rob = msg

#call back function to sub_rob2 to store the position of 2nd robot
def callbackrob2(msg):
    global Pose_rob2
    Pose_rob2 = msg

#call back function to sub_rob3 to store the position of 3rd robot
def callbackrob3(msg):
    global Pose_rob3
    Pose_rob3 = msg

#call this function when status = 1 to combute the robots seen by the robot
def control():
	global pub_rob
	global Pose_rob
	global Pose_rob2
	global Pose_rob3
	global status
	global rob_seen
	
	#put the orientation of the robot in the quaternion variable
	quaternion = (
		Pose_rob.orientation.x,
		Pose_rob.orientation.y,
		Pose_rob.orientation.z,
		Pose_rob.orientation.w)
	
	#transform the orientation from quaternion to euler
	euler = transformations.euler_from_quaternion(quaternion)
	
	#stores the orientation in the form of radians
	yaw_ = euler[2]
	
	#simulates the 3 angles of the triangle that represents the field of view of the robot wrt to 0 rad
	ROB_FIELD_OF_VIEW = [[Pose_rob.position.x , Pose_rob.position.y ], 
	[Pose_rob.position.x + CAM_FOCAL_LENGTH , Pose_rob.position.y - Pose_rob.position.x * math.tan(CAM_ANGLE_OF_VIEW/2)] ,
	[Pose_rob.position.x + CAM_FOCAL_LENGTH , Pose_rob.position.y + Pose_rob.position.x * math.tan(CAM_ANGLE_OF_VIEW/2)]]
	
	#performs rotation matrix on the field of view triangle wrt to the origin
	ROB_FIELD_OF_VIEW_R =[
				[ROB_FIELD_OF_VIEW[0][0] , ROB_FIELD_OF_VIEW[0][1]],
				[(ROB_FIELD_OF_VIEW[1][0]-Pose_rob.position.x) * math.cos(yaw_) - (ROB_FIELD_OF_VIEW[1][1]-Pose_rob.position.y) *math.sin(yaw_),
				(ROB_FIELD_OF_VIEW[1][0]-Pose_rob.position.x) * math.sin(yaw_) + (ROB_FIELD_OF_VIEW[1][1]-Pose_rob.position.y) *math.cos(yaw_)],
				[(ROB_FIELD_OF_VIEW[2][0]-Pose_rob.position.x) * math.cos(yaw_) - (ROB_FIELD_OF_VIEW[2][1]-Pose_rob.position.y) *math.sin(yaw_),
				(ROB_FIELD_OF_VIEW[2][0]-Pose_rob.position.x) * math.sin(yaw_) + (ROB_FIELD_OF_VIEW[2][1]-Pose_rob.position.y) *math.cos(yaw_)]
				]
	
	#performs transitional matrix on the field of view triangle to transform it from the origin to the robot calculating the robots seen
	ROB_FIELD_OF_VIEW_T =[
				[ROB_FIELD_OF_VIEW_R[0][0] , ROB_FIELD_OF_VIEW_R[0][1]],
				[(ROB_FIELD_OF_VIEW_R[1][0]+Pose_rob.position.x),(ROB_FIELD_OF_VIEW_R[1][1]+Pose_rob.position.y)],
				[(ROB_FIELD_OF_VIEW_R[2][0]+Pose_rob.position.x),(ROB_FIELD_OF_VIEW_R[2][1]+Pose_rob.position.y)]
				]
	
	#calculates if the 2nd robot is to the right or the left of the edges of the triangle. if +ve then it is to the right of the line
	#if -ve then it is to the left of the line. so if the D1,D2,D3 are +ve then the robot is inside the triangle thus it is considered
	#to be seen by the robot.
	D1 = (ROB_FIELD_OF_VIEW_T[1][0] - ROB_FIELD_OF_VIEW_T[0][0]) * (Pose_rob2.position.y - ROB_FIELD_OF_VIEW_T[0][1]) - (Pose_rob2.position.x - ROB_FIELD_OF_VIEW_T[0][0]) * (ROB_FIELD_OF_VIEW_T[1][1] - ROB_FIELD_OF_VIEW_T[0][1])
	D2 = (ROB_FIELD_OF_VIEW_T[2][0] - ROB_FIELD_OF_VIEW_T[1][0]) * (Pose_rob2.position.y - ROB_FIELD_OF_VIEW_T[1][1]) - (Pose_rob2.position.x -ROB_FIELD_OF_VIEW_T[1][0]) * (ROB_FIELD_OF_VIEW_T[2][1] - ROB_FIELD_OF_VIEW_T[1][1])
	D3 = (ROB_FIELD_OF_VIEW_T[0][0] - ROB_FIELD_OF_VIEW_T[2][0]) * (Pose_rob2.position.y - ROB_FIELD_OF_VIEW_T[2][1]) - (Pose_rob2.position.x -ROB_FIELD_OF_VIEW_T[2][0]) * (ROB_FIELD_OF_VIEW_T[0][1] - ROB_FIELD_OF_VIEW_T[2][1])
	
	#calculates if the 3rd robot is to the right or the left of the edges of the triangle. if +ve then it is to the right of the line
	#if -ve then it is to the left of the line. so if the D1,D2,D3 are +ve then the robot is inside the triangle thus it is considered
	#to be seen by the robot.
	D4 = (ROB_FIELD_OF_VIEW_T[1][0] - ROB_FIELD_OF_VIEW_T[0][0]) * (Pose_rob3.position.y - ROB_FIELD_OF_VIEW_T[0][1]) - (Pose_rob3.position.x -ROB_FIELD_OF_VIEW_T[0][0]) * (ROB_FIELD_OF_VIEW_T[1][1] - ROB_FIELD_OF_VIEW_T[0][1])
	D5 = (ROB_FIELD_OF_VIEW_T[2][0] - ROB_FIELD_OF_VIEW_T[1][0]) * (Pose_rob3.position.y - ROB_FIELD_OF_VIEW_T[1][1]) - (Pose_rob3.position.x -ROB_FIELD_OF_VIEW_T[1][0]) * (ROB_FIELD_OF_VIEW_T[2][1] - ROB_FIELD_OF_VIEW_T[1][1])
	D6 = (ROB_FIELD_OF_VIEW_T[0][0] - ROB_FIELD_OF_VIEW_T[2][0]) * (Pose_rob3.position.y - ROB_FIELD_OF_VIEW_T[2][1]) - (Pose_rob3.position.x -ROB_FIELD_OF_VIEW_T[2][0]) * (ROB_FIELD_OF_VIEW_T[0][1] - ROB_FIELD_OF_VIEW_T[2][1])
	
	#checks if the 3 variables are +ve then increases the number of robots seen by one.
	if(D1 > 0 and D2 > 0 and D3 > 0):
			rob_seen += 1
	if(D4 > 0 and D5 > 0 and D6 > 0):
			rob_seen += 1
	#print(D1 , D2, D3)
	#print(D4 , D5, D6)
	print(rob_seen)
	
	#change status to 0 so it doesn't keeps calculating the robots seen while they are moving
	status = 0
	
	
def main():
	global sub_rob
	global pub_rob
	global pub_rob2
	global pub_rob3
	global status
	global rob_seen
	
	rospy.init_node('Peer_Rob')
	
	sub_rob = rospy.Subscriber('/pose',Pose,callbackrob1)
	sub_rob2 = rospy.Subscriber('/pose2',Pose,callbackrob2)
	sub_rob3 = rospy.Subscriber('/pose3',Pose,callbackrob3)
	pub_rob = rospy.Publisher('/robots_seen',Int8,queue_size=1)
	
	rate = rospy.Rate(20)
	while not rospy.is_shutdown() :
		rate.sleep()#delay to wait until poses of the robots are published else the calculations may be wrong
		rate.sleep()
		rate.sleep()
		rate.sleep()
		if(status == 1):	
			control()
		pub_rob.publish(rob_seen) #keeps publishing the robots seen by the robot on the topic of the robot
	else:
		rospy.logerr('Unknown state!')
	
	rospy.spin()
  
if __name__ == "__main__":
    main()
