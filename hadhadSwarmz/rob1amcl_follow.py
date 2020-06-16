#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
#posewithcovariance bta3t mcl pose
from geometry_msgs.msg import Twist, Point ,PoseWithCovarianceStamped,PoseStamped
from nav_msgs.msg import Odometry #to know robot pos
from tf import transformations # gowah conversion quarention to euler
#trajector msg
from visualization_msgs.msg import MarkerArray
import numpy as np
import math

from nav_msgs.srv import GetMap



position_ = PoseWithCovarianceStamped()
yaw_=0

#publisher to cmd_vel
pub_rob2 = None
pub_rob3 = None

shape_dist = 1.0

def callbackrob1g(msg):
    global position_
    global yaw_
    global shape_dist
    # tester = PoseStamped()
    # tester2=PoseWithCovarianceStamped()
    #1st pose is post with covariance
    position_ = msg
    
    goal_rob2 = PoseStamped()
    goal_rob3 = PoseStamped()
    goal_rob2.header.frame_id = position_.header.frame_id
    goal_rob3.header.frame_id = position_.header.frame_id

    goal_rob2.pose.position.x = position_.pose.pose.position.x + shape_dist
    goal_rob2.pose.position.y = position_.pose.pose.position.y - shape_dist
    
    goal_rob2.pose.orientation.z = position_.pose.pose.orientation.z
    goal_rob2.pose.orientation.w = position_.pose.pose.orientation.w

    goal_rob3.pose.position.x = position_.pose.pose.position.x + shape_dist
    goal_rob3.pose.position.y = position_.pose.pose.position.y + shape_dist
    goal_rob3.pose.orientation.z = position_.pose.pose.orientation.z
    goal_rob3.pose.orientation.w = position_.pose.pose.orientation.w  

    pub_rob2.publish(goal_rob2)
    pub_rob3.publish(goal_rob3)
   

def main():
    global pub_rob2
    global pub_rob3
    global rate
    rospy.init_node('hadhad_swarmz')

    pub_rob2 = rospy.Publisher('/rob2/move_base_simple/goal',PoseStamped,queue_size=1000)
    pub_rob3 = rospy.Publisher('/rob3/move_base_simple/goal',PoseStamped,queue_size=1000)

    sub_rob1g = rospy.Subscriber('/rob1/amcl_pose',PoseWithCovarianceStamped,callbackrob1g)

    rate = rospy.Rate(20)
    # while not rospy.is_shutdown() :
    #     if state_ == 0:
    #         fix_yaw(desired_position_)
    #     elif state_ == 1:
    #         go_straight_ahead(desired_position_)
    #     elif state_ == 2:
    #         done()
    #     else:
    #          rospy.logerr('Unknown state!')
        
    #     rate.sleep()
    rospy.spin()
  
if __name__ == "__main__":
    main()

