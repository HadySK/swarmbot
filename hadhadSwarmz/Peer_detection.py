#!/usr/bin/env python

import rospy
#posewithcovariance bta3t mcl pose
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8
from tf import transformations # gowah conversion quarention to euler
import numpy as np
import math

CAM_FOCAL_LENGTH = 3.33
CAM_ANGLE_OF_VIEW = 2 * math.pi / 3

#publisher to cmd_vel
sub_rob = None
sub_rob2 = None
sub_rob3 = None
pub_rob = None
pub_led  = None
pub_fol1 = None
pub_fol2 = None
Pose_rob = Pose()
Pose_rob2 = Pose()
Pose_rob3 = Pose()	

rob_seen = 0
status = 1
shape_dist = 1.5

def callbackrob1(msg):
    global Pose_rob
    Pose_rob = msg

def callbackrob2(msg):
    global Pose_rob2
    Pose_rob2 = msg

def callbackrob3(msg):
    global Pose_rob3
    Pose_rob3 = msg

def control():
	global pub_rob
	global Pose_rob
	global Pose_rob2
	global Pose_rob3
	global status
	global rob_seen
	
	quaternion = (
		Pose_rob.orientation.x,
		Pose_rob.orientation.y,
		Pose_rob.orientation.z,
		Pose_rob.orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	yaw_ = euler[2]
	
	ROB_FIELD_OF_VIEW = [[Pose_rob.position.x , Pose_rob.position.y ], 
	[Pose_rob.position.x + CAM_FOCAL_LENGTH , Pose_rob.position.y - Pose_rob.position.x * math.tan(CAM_ANGLE_OF_VIEW/2)] ,
	[Pose_rob.position.x + CAM_FOCAL_LENGTH , Pose_rob.position.y + Pose_rob.position.x * math.tan(CAM_ANGLE_OF_VIEW/2)]]
	
	ROB_FIELD_OF_VIEW_R =[
				[ROB_FIELD_OF_VIEW[0][0] , ROB_FIELD_OF_VIEW[0][1]],
				[(ROB_FIELD_OF_VIEW[1][0]-Pose_rob.position.x) * math.cos(yaw_) - (ROB_FIELD_OF_VIEW[1][1]-Pose_rob.position.y) *math.sin(yaw_),
				(ROB_FIELD_OF_VIEW[1][0]-Pose_rob.position.x) * math.sin(yaw_) + (ROB_FIELD_OF_VIEW[1][1]-Pose_rob.position.y) *math.cos(yaw_)],
				[(ROB_FIELD_OF_VIEW[2][0]-Pose_rob.position.x) * math.cos(yaw_) - (ROB_FIELD_OF_VIEW[2][1]-Pose_rob.position.y) *math.sin(yaw_),
				(ROB_FIELD_OF_VIEW[2][0]-Pose_rob.position.x) * math.sin(yaw_) + (ROB_FIELD_OF_VIEW[2][1]-Pose_rob.position.y) *math.cos(yaw_)]
				]
	ROB_FIELD_OF_VIEW_T =[
				[ROB_FIELD_OF_VIEW_R[0][0] , ROB_FIELD_OF_VIEW_R[0][1]],
				[(ROB_FIELD_OF_VIEW_R[1][0]+Pose_rob.position.x),(ROB_FIELD_OF_VIEW_R[1][1]+Pose_rob.position.y)],
				[(ROB_FIELD_OF_VIEW_R[2][0]+Pose_rob.position.x),(ROB_FIELD_OF_VIEW_R[2][1]+Pose_rob.position.y)]
				]
				
	D1 = (ROB_FIELD_OF_VIEW_T[1][0] - ROB_FIELD_OF_VIEW_T[0][0]) * (Pose_rob2.position.y - ROB_FIELD_OF_VIEW_T[0][1]) - (Pose_rob2.position.x - ROB_FIELD_OF_VIEW_T[0][0]) * (ROB_FIELD_OF_VIEW_T[1][1] - ROB_FIELD_OF_VIEW_T[0][1])
	D2 = (ROB_FIELD_OF_VIEW_T[2][0] - ROB_FIELD_OF_VIEW_T[1][0]) * (Pose_rob2.position.y - ROB_FIELD_OF_VIEW_T[1][1]) - (Pose_rob2.position.x -ROB_FIELD_OF_VIEW_T[1][0]) * (ROB_FIELD_OF_VIEW_T[2][1] - ROB_FIELD_OF_VIEW_T[1][1])
	D3 = (ROB_FIELD_OF_VIEW_T[0][0] - ROB_FIELD_OF_VIEW_T[2][0]) * (Pose_rob2.position.y - ROB_FIELD_OF_VIEW_T[2][1]) - (Pose_rob2.position.x -ROB_FIELD_OF_VIEW_T[2][0]) * (ROB_FIELD_OF_VIEW_T[0][1] - ROB_FIELD_OF_VIEW_T[2][1])
	
	D4 = (ROB_FIELD_OF_VIEW_T[1][0] - ROB_FIELD_OF_VIEW_T[0][0]) * (Pose_rob3.position.y - ROB_FIELD_OF_VIEW_T[0][1]) - (Pose_rob3.position.x -ROB_FIELD_OF_VIEW_T[0][0]) * (ROB_FIELD_OF_VIEW_T[1][1] - ROB_FIELD_OF_VIEW_T[0][1])
	D5 = (ROB_FIELD_OF_VIEW_T[2][0] - ROB_FIELD_OF_VIEW_T[1][0]) * (Pose_rob3.position.y - ROB_FIELD_OF_VIEW_T[1][1]) - (Pose_rob3.position.x -ROB_FIELD_OF_VIEW_T[1][0]) * (ROB_FIELD_OF_VIEW_T[2][1] - ROB_FIELD_OF_VIEW_T[1][1])
	D6 = (ROB_FIELD_OF_VIEW_T[0][0] - ROB_FIELD_OF_VIEW_T[2][0]) * (Pose_rob3.position.y - ROB_FIELD_OF_VIEW_T[2][1]) - (Pose_rob3.position.x -ROB_FIELD_OF_VIEW_T[2][0]) * (ROB_FIELD_OF_VIEW_T[0][1] - ROB_FIELD_OF_VIEW_T[2][1])
	
	if(D1 > 0 and D2 > 0 and D3 > 0):
			rob_seen += 1
	if(D4 > 0 and D5 > 0 and D6 > 0):
			rob_seen += 1
	print(rob_seen)
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
		rate.sleep()
		rate.sleep()
		rate.sleep()
		rate.sleep()
		if(status == 1):	
			control()
		pub_rob.publish(rob_seen)
	else:
		rospy.logerr('Unknown state!')
	
	rospy.spin()
  
if __name__ == "__main__":
    main()
