#!/usr/bin/env python
# Lucas Walter
# Configure a CameraInfo from a dynamic reconfigure interface

import rospy
import yaml
from control_msgs.msg import JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory

if __name__ == '__main__':
    rospy.init_node('dr_camera_info')
    pub = rospy.Publisher("FollowJointTrajectoryActionGoal",JointTrajectoryGoal , queue_size=10)
    trajectory = "../yaml/traj_home_goal1.yaml"    
    with open(trajectory, "r") as yaml_file:
        data = yaml.load(yaml_file)
        traj_msg = JointTrajectory()
#        print traj_msg
        

        
#        traj_msg = data["trajectory"]
        print traj_msg
    
    
    

    rospy.spin()