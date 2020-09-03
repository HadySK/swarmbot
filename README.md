# hswarmros
Swarm robots simulation in gazebo and ROS

Work in progress

1) con_robots.launch >> to launch simulation environment
#make sure it prints on the terminal 'odom recieved!', if not terminate in the terminal(ctrl+c) and type 'killall gzserver' in the terminal then relaunch.

2) Peer_detection.launch >> to launch the peer detection algorithm

3) Leader_selection.launch >> to launch the leader selection algorithm

4) Narrow_path_detection.launch >> to launch the narrow path detection algorithm

#make sure to give the navigation goal for the first time on RVIZ before running the formation algorithm

5) Formation.launch >> to launch the Formation algorithm

## for better formation make sure to decrease the velocity of the leader robot in the planner.yaml in param folder.


---prerequisites---

Navigation stack
Map server
Gazebo
Rviz
