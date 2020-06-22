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

CAM_FOCAL_LENGTH = 3.33
CAM_ANGLE_OF_VIEW = 2 * math.pi / 3

#publisher to cmd_vel
sub_rob1 = None
sub_rob2 = None
sub_rob3 = None
pub_rob1 = None
pub_rob2 = None
pub_rob3 = None
pub_led  = None
pub_fol1 = None
pub_fol2 = None
Pose_rob1 = Pose()
Pose_rob2 = Pose()
Pose_rob3 = Pose()	

leader = 0
follower1 = 0
follower2 = 0
status = 1
shape_dist = 1.5

def callbackrob1(msg):
    global Pose_rob1
    Pose_rob1 = msg

def callbackrob2(msg):
    global Pose_rob2
    Pose_rob2 = msg

def callbackrob3(msg):
    global Pose_rob3
    Pose_rob3 = msg 

def control():
	global pub_rob1
	global Pose_rob1
	global pub_rob2
	global Pose_rob2
	global pub_rob3
	global Pose_rob3
	global pub_led
	global pub_fol1
	global pub_fol2
	global status
	global leader
	global follower1
	global follower2
	
	pose = [Pose_rob1 , Pose_rob2 , Pose_rob3]
	rob_seen = [0 , 0 , 0]
	distance = 1000
	
	quaternion = (
		Pose_rob1.orientation.x,
		Pose_rob1.orientation.y,
		Pose_rob1.orientation.z,
		Pose_rob1.orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	yaw_ = euler[2]
	
	ROB1_FIELD_OF_VIEW = [[Pose_rob1.position.x , Pose_rob1.position.y ], 
	[Pose_rob1.position.x + CAM_FOCAL_LENGTH , Pose_rob1.position.y - Pose_rob1.position.x * math.tan(CAM_ANGLE_OF_VIEW/2)] ,
	[Pose_rob1.position.x + CAM_FOCAL_LENGTH , Pose_rob1.position.y + Pose_rob1.position.x * math.tan(CAM_ANGLE_OF_VIEW/2)]]
	
	ROB1_FIELD_OF_VIEWx =[
				[ROB1_FIELD_OF_VIEW[0][0] , ROB1_FIELD_OF_VIEW[0][1]],
				[ROB1_FIELD_OF_VIEW[1][0] * math.cos(yaw_) - ROB1_FIELD_OF_VIEW[1][1] *math.sin(yaw_),
				ROB1_FIELD_OF_VIEW[1][0] * math.sin(yaw_) + ROB1_FIELD_OF_VIEW[1][1] *math.cos(yaw_)],
				[ROB1_FIELD_OF_VIEW[2][0] * math.cos(yaw_) - ROB1_FIELD_OF_VIEW[2][1] *math.sin(yaw_),
				ROB1_FIELD_OF_VIEW[2][0] * math.sin(yaw_) + ROB1_FIELD_OF_VIEW[2][1] *math.cos(yaw_)]
				]
				
	D1 = (ROB1_FIELD_OF_VIEWx[1][0] - ROB1_FIELD_OF_VIEWx[0][0]) * (Pose_rob2.position.y - ROB1_FIELD_OF_VIEWx[0][1]) - (Pose_rob2.position.x -ROB1_FIELD_OF_VIEWx[0][0]) * (ROB1_FIELD_OF_VIEWx[1][1] - ROB1_FIELD_OF_VIEWx[0][1])
	D2 = (ROB1_FIELD_OF_VIEWx[2][0] - ROB1_FIELD_OF_VIEWx[1][0]) * (Pose_rob2.position.y - ROB1_FIELD_OF_VIEWx[1][1]) - (Pose_rob2.position.x -ROB1_FIELD_OF_VIEWx[1][0]) * (ROB1_FIELD_OF_VIEWx[2][1] - ROB1_FIELD_OF_VIEWx[1][1])
	D3 = (ROB1_FIELD_OF_VIEWx[0][0] - ROB1_FIELD_OF_VIEWx[2][0]) * (Pose_rob2.position.y - ROB1_FIELD_OF_VIEWx[2][1]) - (Pose_rob2.position.x -ROB1_FIELD_OF_VIEWx[2][0]) * (ROB1_FIELD_OF_VIEWx[0][1] - ROB1_FIELD_OF_VIEWx[2][1])
	
	D4 = (ROB1_FIELD_OF_VIEWx[1][0] - ROB1_FIELD_OF_VIEWx[0][0]) * (Pose_rob3.position.y - ROB1_FIELD_OF_VIEWx[0][1]) - (Pose_rob3.position.x -ROB1_FIELD_OF_VIEWx[0][0]) * (ROB1_FIELD_OF_VIEWx[1][1] - ROB1_FIELD_OF_VIEWx[0][1])
	D5 = (ROB1_FIELD_OF_VIEWx[2][0] - ROB1_FIELD_OF_VIEWx[1][0]) * (Pose_rob3.position.y - ROB1_FIELD_OF_VIEWx[1][1]) - (Pose_rob3.position.x -ROB1_FIELD_OF_VIEWx[1][0]) * (ROB1_FIELD_OF_VIEWx[2][1] - ROB1_FIELD_OF_VIEWx[1][1])
	D6 = (ROB1_FIELD_OF_VIEWx[0][0] - ROB1_FIELD_OF_VIEWx[2][0]) * (Pose_rob3.position.y - ROB1_FIELD_OF_VIEWx[2][1]) - (Pose_rob3.position.x -ROB1_FIELD_OF_VIEWx[2][0]) * (ROB1_FIELD_OF_VIEWx[0][1] - ROB1_FIELD_OF_VIEWx[2][1])
	
	if(D1 > 0 and D2 > 0 and D3 > 0):
			rob_seen[0] += 1
	if(D4 > 0 and D5 > 0 and D6 > 0):
			rob_seen[0] += 1
	pub_rob1.publish(rob_seen[0])
	
	#print(D1,D2,D3)
	#print(D4,D5,D6)
	
	quaternion = (
		Pose_rob2.orientation.x,
		Pose_rob2.orientation.y,
		Pose_rob2.orientation.z,
		Pose_rob2.orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	yaw_ = euler[2]
	
	ROB2_FIELD_OF_VIEW = [[Pose_rob2.position.x , Pose_rob2.position.y ], 
	[Pose_rob2.position.x + CAM_FOCAL_LENGTH , Pose_rob2.position.y - Pose_rob2.position.x * math.tan(CAM_ANGLE_OF_VIEW/2)] ,
	[Pose_rob2.position.x + CAM_FOCAL_LENGTH , Pose_rob2.position.y + Pose_rob2.position.x * math.tan(CAM_ANGLE_OF_VIEW/2)]]
	
	ROB2_FIELD_OF_VIEWx =[
				[ROB2_FIELD_OF_VIEW[0][0] , ROB2_FIELD_OF_VIEW[0][1]],
				[ROB2_FIELD_OF_VIEW[1][0] * math.cos(yaw_) - ROB2_FIELD_OF_VIEW[1][1] *math.sin(yaw_),
				ROB2_FIELD_OF_VIEW[1][0] * math.sin(yaw_) + ROB2_FIELD_OF_VIEW[1][1] *math.cos(yaw_)],
				[ROB2_FIELD_OF_VIEW[2][0] * math.cos(yaw_) - ROB2_FIELD_OF_VIEW[2][1] *math.sin(yaw_),
				ROB2_FIELD_OF_VIEW[2][0] * math.sin(yaw_) + ROB2_FIELD_OF_VIEW[2][1] *math.cos(yaw_)]
				]
				
	D1 = (ROB2_FIELD_OF_VIEWx[1][0] - ROB2_FIELD_OF_VIEWx[0][0]) * (Pose_rob1.position.y - ROB2_FIELD_OF_VIEWx[0][1]) - (Pose_rob1.position.x -ROB2_FIELD_OF_VIEWx[0][0]) * (ROB2_FIELD_OF_VIEWx[1][1] - ROB2_FIELD_OF_VIEWx[0][1])
	D2 = (ROB2_FIELD_OF_VIEWx[2][0] - ROB2_FIELD_OF_VIEWx[1][0]) * (Pose_rob1.position.y - ROB2_FIELD_OF_VIEWx[1][1]) - (Pose_rob1.position.x -ROB2_FIELD_OF_VIEWx[1][0]) * (ROB2_FIELD_OF_VIEWx[2][1] - ROB2_FIELD_OF_VIEWx[1][1])
	D3 = (ROB2_FIELD_OF_VIEWx[0][0] - ROB2_FIELD_OF_VIEWx[2][0]) * (Pose_rob1.position.y - ROB2_FIELD_OF_VIEWx[2][1]) - (Pose_rob1.position.x -ROB2_FIELD_OF_VIEWx[2][0]) * (ROB2_FIELD_OF_VIEWx[0][1] - ROB2_FIELD_OF_VIEWx[2][1])
	
	D4 = (ROB2_FIELD_OF_VIEWx[1][0] - ROB2_FIELD_OF_VIEWx[0][0]) * (Pose_rob3.position.y - ROB2_FIELD_OF_VIEWx[0][1]) - (Pose_rob3.position.x -ROB2_FIELD_OF_VIEWx[0][0]) * (ROB2_FIELD_OF_VIEWx[1][1] - ROB2_FIELD_OF_VIEWx[0][1])
	D5 = (ROB2_FIELD_OF_VIEWx[2][0] - ROB2_FIELD_OF_VIEWx[1][0]) * (Pose_rob3.position.y - ROB2_FIELD_OF_VIEWx[1][1]) - (Pose_rob3.position.x -ROB2_FIELD_OF_VIEWx[1][0]) * (ROB2_FIELD_OF_VIEWx[2][1] - ROB2_FIELD_OF_VIEWx[1][1])
	D6 = (ROB2_FIELD_OF_VIEWx[0][0] - ROB2_FIELD_OF_VIEWx[2][0]) * (Pose_rob3.position.y - ROB2_FIELD_OF_VIEWx[2][1]) - (Pose_rob3.position.x -ROB2_FIELD_OF_VIEWx[2][0]) * (ROB2_FIELD_OF_VIEWx[0][1] - ROB2_FIELD_OF_VIEWx[2][1])
	
	if(D1 > 0 and D2 > 0 and D3 > 0):
			rob_seen[1] += 1
	if(D4 > 0 and D5 > 0 and D6 > 0):
			rob_seen[1] += 1
	pub_rob2.publish(rob_seen[1])
	
	#print(D1,D2,D3)
	#print(D4,D5,D6)
	
	quaternion = (
		Pose_rob3.orientation.x,
		Pose_rob3.orientation.y,
		Pose_rob3.orientation.z,
		Pose_rob3.orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	yaw_ = euler[2]
	
	ROB3_FIELD_OF_VIEW = [[Pose_rob3.position.x , Pose_rob3.position.y ], 
	[Pose_rob3.position.x + CAM_FOCAL_LENGTH , Pose_rob3.position.y - Pose_rob3.position.x * math.tan(CAM_ANGLE_OF_VIEW/2)] ,
	[Pose_rob3.position.x + CAM_FOCAL_LENGTH , Pose_rob3.position.y + Pose_rob3.position.x * math.tan(CAM_ANGLE_OF_VIEW/2)]]
	
	ROB3_FIELD_OF_VIEWx =[
				[ROB3_FIELD_OF_VIEW[0][0] , ROB3_FIELD_OF_VIEW[0][1]],
				[ROB3_FIELD_OF_VIEW[1][0] * math.cos(yaw_) - ROB3_FIELD_OF_VIEW[1][1] *math.sin(yaw_),
				ROB3_FIELD_OF_VIEW[1][0] * math.sin(yaw_) + ROB3_FIELD_OF_VIEW[1][1] *math.cos(yaw_)],
				[ROB3_FIELD_OF_VIEW[2][0] * math.cos(yaw_) - ROB3_FIELD_OF_VIEW[2][1] *math.sin(yaw_),
				ROB3_FIELD_OF_VIEW[2][0] * math.sin(yaw_) + ROB3_FIELD_OF_VIEW[2][1] *math.cos(yaw_)]
				]
				
	D1 = (ROB3_FIELD_OF_VIEWx[1][0] - ROB3_FIELD_OF_VIEWx[0][0]) * (Pose_rob1.position.y - ROB3_FIELD_OF_VIEWx[0][1]) - (Pose_rob1.position.x -ROB3_FIELD_OF_VIEWx[0][0]) * (ROB3_FIELD_OF_VIEWx[1][1] - ROB3_FIELD_OF_VIEWx[0][1])
	D2 = (ROB3_FIELD_OF_VIEWx[2][0] - ROB3_FIELD_OF_VIEWx[1][0]) * (Pose_rob1.position.y - ROB3_FIELD_OF_VIEWx[1][1]) - (Pose_rob1.position.x -ROB3_FIELD_OF_VIEWx[1][0]) * (ROB3_FIELD_OF_VIEWx[2][1] - ROB3_FIELD_OF_VIEWx[1][1])
	D3 = (ROB3_FIELD_OF_VIEWx[0][0] - ROB3_FIELD_OF_VIEWx[2][0]) * (Pose_rob1.position.y - ROB3_FIELD_OF_VIEWx[2][1]) - (Pose_rob1.position.x -ROB3_FIELD_OF_VIEWx[2][0]) * (ROB3_FIELD_OF_VIEWx[0][1] - ROB3_FIELD_OF_VIEWx[2][1])
	
	D4 = (ROB3_FIELD_OF_VIEWx[1][0] - ROB3_FIELD_OF_VIEWx[0][0]) * (Pose_rob2.position.y - ROB3_FIELD_OF_VIEWx[0][1]) - (Pose_rob2.position.x -ROB3_FIELD_OF_VIEWx[0][0]) * (ROB3_FIELD_OF_VIEWx[1][1] - ROB3_FIELD_OF_VIEWx[0][1])
	D5 = (ROB3_FIELD_OF_VIEWx[2][0] - ROB3_FIELD_OF_VIEWx[1][0]) * (Pose_rob2.position.y - ROB3_FIELD_OF_VIEWx[1][1]) - (Pose_rob2.position.x -ROB3_FIELD_OF_VIEWx[1][0]) * (ROB3_FIELD_OF_VIEWx[2][1] - ROB3_FIELD_OF_VIEWx[1][1])
	D6 = (ROB3_FIELD_OF_VIEWx[0][0] - ROB3_FIELD_OF_VIEWx[2][0]) * (Pose_rob2.position.y - ROB3_FIELD_OF_VIEWx[2][1]) - (Pose_rob2.position.x -ROB3_FIELD_OF_VIEWx[2][0]) * (ROB3_FIELD_OF_VIEWx[0][1] - ROB3_FIELD_OF_VIEWx[2][1])
	
	if(D1 > 0 and D2 > 0 and D3 > 0):
			rob_seen[2] += 1
	if(D4 > 0 and D5 > 0 and D6 > 0):
			rob_seen[2] += 1
	pub_rob3.publish(rob_seen[2])
	
	#print(D1,D2,D3)
	#print(D4,D5,D6)
	
	for i in range(0,3):
		if(rob_seen[i] > rob_seen[leader]):
			leader = i
	
	followerx = pose[leader].position.x - shape_dist*math.sin((math.pi/3 + yaw_))
    	followery = pose[leader].position.y + shape_dist*math.cos((math.pi/3 + yaw_))
	for i in range(0,3):
		if(i != leader):
			if(math.sqrt(math.pow(followerx - pose[i].position.x , 2) + math.pow(followery - pose[i].position.y , 2)) < distance):
				follwer1 = i
	for i in range(0,3):
		if(i != leader and i != follower1):
			follower2 = i
	
	status = 0
	
def main():
	global sub_rob1
	global sub_rob2
	global sub_rob3
	global pub_rob1
	global pub_rob2
	global pub_rob3
	global pub_led
	global pub_fol1
	global pub_fol2
	global status
	global leader
	global follower1
	global follower2
	
	rospy.init_node('Leader')
	
	sub_rob1 = rospy.Subscriber('/rob1/pose',Pose,callbackrob1)
	sub_rob2 = rospy.Subscriber('/rob2/pose',Pose,callbackrob2)
	sub_rob3 = rospy.Subscriber('/rob3/pose',Pose,callbackrob3)
	
	pub_rob1 = rospy.Publisher('/rob1/robots_seen',Int8,queue_size=1)
	pub_rob2 = rospy.Publisher('/rob2/robots_seen',Int8,queue_size=1)
	pub_rob3 = rospy.Publisher('/rob3/robots_seen',Int8,queue_size=1)
	pub_led = rospy.Publisher('/leader',Int8,queue_size=1)
	pub_fol1 = rospy.Publisher('/follower1',Int8,queue_size=1)
	pub_fol2 = rospy.Publisher('/follower2',Int8,queue_size=1)
	
	rate = rospy.Rate(20)
	while not rospy.is_shutdown() :
		rate.sleep()
		rate.sleep()
		if(status == 1):	
			control()
		pub_led.publish(leader)
		pub_fol1.publish(follower1)
		pub_fol2.publish(follower2)
	else:
		rospy.logerr('Unknown state!')
	
	rospy.spin()
  
if __name__ == "__main__":
    main()

