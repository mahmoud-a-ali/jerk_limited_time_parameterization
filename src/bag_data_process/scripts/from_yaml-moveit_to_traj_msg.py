#!/usr/bin/env python

import rospy
#import numpy as np

from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryActionGoal
from control_msgs.msg import JointTrajectoryGoal

pub_msg = JointTrajectory()
pub = rospy.Publisher('/joint_trajectory', JointTrajectory, queue_size=10)

def moveit_goal_callback( msg ):
    n_pts = len(msg.goal.trajectory.points)
    n_jts = len(msg.goal.trajectory.points[0].positions)
    rospy.loginfo( "jts: %s", n_jts)    
    pub_msg =  msg.goal.trajectory
    pub.publish(pub_msg)



def yaml_points_callback( msg ):
    n_pts = len(msg.trajectory.points)
    n_jts = len(msg.trajectory.points[0].positions)
    rospy.loginfo( "jts: %s", n_jts)    
    pub_msg =  msg.trajectory
    pub.publish(pub_msg)

    
def pub_sub():
    rospy.init_node('yaml-moveit_to_joint_traj_msg')
    #sub to follow_joint_trajectory/goal sent from moveit to ur5 (gazebo) 
    rospy.Subscriber("/arm_controller/follow_joint_trajectory/goal", FollowJointTrajectoryActionGoal, moveit_goal_callback)
    rospy.Subscriber("/joint_traj_goal_yaml", JointTrajectoryGoal, yaml_points_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        pub_sub()
    except rospy.ROSInterruptException:
        pass
