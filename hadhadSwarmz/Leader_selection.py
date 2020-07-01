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
sub_rob_pos = None
sub_rob2_pos = None
sub_rob3_pos = None
pub_led  = None
pub_fol1 = None
pub_fol2 = None	

Pose_rob = Pose()
Pose_rob2 = Pose()
Pose_rob3 = Pose()

rob1_seen = [0]
rob2_seen = [0]
rob3_seen = [0]
robs_seen = [rob1_seen , rob2_seen , rob3_seen]

leader = 0
follower1 = 0
follower2 = 0
status = 1
shape_dist = 1.5

def callbackrob1(msg):
    global rob1_seen
    rob1_seen[0] = msg.data

def callbackrob2(msg):
    global rob2_seen
    rob2_seen[0] = msg.data

def callbackrob3(msg):
    global rob3_seen
    rob3_seen[0] = msg.data

def callbackrob4(msg):
    global Pose_rob
    Pose_rob = msg
    
def callbackrob5(msg):
    global Pose_rob2
    Pose_rob2 = msg

def callbackrob6(msg):
    global Pose_rob3
    Pose_rob3 = msg
    
def control():
	global robs_seen
	global Pose_rob
	global Pose_rob2
	global Pose_rob3
	global leader
	global follower1
	global follower2
	global status
	
	pose = [Pose_rob , Pose_rob2 , Pose_rob3]
	yaw_ = [0,0,0]
	distance = 1000
	print(robs_seen)
	quaternion = (
		Pose_rob.orientation.x,
		Pose_rob.orientation.y,
		Pose_rob.orientation.z,
		Pose_rob.orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	yaw_[0] = euler[2]
	
	quaternion = (
		Pose_rob2.orientation.x,
		Pose_rob2.orientation.y,
		Pose_rob2.orientation.z,
		Pose_rob2.orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	yaw_[1] = euler[2]
	
	quaternion = (
		Pose_rob3.orientation.x,
		Pose_rob3.orientation.y,
		Pose_rob3.orientation.z,
		Pose_rob3.orientation.w)
	euler = transformations.euler_from_quaternion(quaternion)
	yaw_[2] = euler[2]
	
	for i in range(0,3):
		if(robs_seen[i] > robs_seen[leader]):
			leader = i
	
	followerx = pose[leader].position.x - shape_dist*math.sin((math.pi/3 + yaw_[leader]))
    	followery = pose[leader].position.y + shape_dist*math.cos((math.pi/3 + yaw_[leader]))
    	
	for i in range(0,3):
		if(i != leader):
			if(math.sqrt(math.pow(followerx - pose[i].position.x , 2) + math.pow(followery - pose[i].position.y , 2)) < distance):
				distance = math.sqrt(math.pow(followerx - pose[i].position.x , 2) + math.pow(followery - pose[i].position.y , 2))
				follower1 = i
	for i in range(0,3):
		if(i != leader and i != follower1):
			follower2 = i
	
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
	
	rate = rospy.Rate(20)
	while not rospy.is_shutdown() :
		rate.sleep()
		rate.sleep()
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

