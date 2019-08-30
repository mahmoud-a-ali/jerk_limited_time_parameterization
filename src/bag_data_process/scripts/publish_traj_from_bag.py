#!/usr/bin/env python
import numpy as np
import rosbag
import sys

import rospy
import numpy as np
import yaml
import os
from control_msgs.msg import FollowJointTrajectoryActionGoal
from trajectory_msgs.msg import JointTrajectory


def get_FJTAGoal_points(msg):
    waypoints = None
    waypoints = msg.goal.trajectory.points
    positions = np.array([np.array(wp.positions) for wp in waypoints])
    velocities = np.array([np.array(wp.velocities) for wp in waypoints])
    accelerations = np.array([np.array(wp.accelerations) for wp in waypoints])
    times = np.array([wp.time_from_start.to_sec() for wp in waypoints])
    return (positions, velocities, accelerations, times)


def pub_FJTAGoal():
    rospy.init_node('publish_traj_from_bag')
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(pub_msg)
        rate.sleep()
    rospy.spin()




if __name__ == '__main__':
    print "initialize seg"
    rospy.init_node('publish_traj_from_bag')
    n_segs = rospy.get_param('~n_segs', 1 )
    pt_i = rospy.get_param('~pt_i', 0 )
    pt_f = rospy.get_param('~pt_f', 0 )
    rate = rospy.Rate(1) # 10hz
    pub_msg = FollowJointTrajectoryActionGoal()
#    pub = rospy.Publisher('/arm_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
    pub = rospy.Publisher('/bag/goal', FollowJointTrajectoryActionGoal, queue_size=10)
    rate.sleep()
    try:
        if(len(sys.argv) < 2):
            print("Err: specify bag as command line arg")
            sys.exit()
        bag = rosbag.Bag(sys.argv[1], 'r')
        print "\n=========>  publish topic:/arm_controller/follow_joint_trajectory/goal "
        for topic, msg, t in bag.read_messages(topics=["/arm_controller/follow_joint_trajectory/goal"]):
#            positions, velocities, accelerations, times = get_FJTAGoal_points(msg)
#            times += t.secs +  t.nsecs/1e9
            pub_msg = msg

        print "n_segs, pt_i, pt_f: {}, {}, {}".format(n_segs, pt_i, pt_f)
        if pt_i==0 and pt_f ==0:
            pub_msg.goal.trajectory.points = pub_msg.goal.trajectory.points[0:n_segs+1]
        else:
            pub_msg.goal.trajectory.points = pub_msg.goal.trajectory.points[pt_i:pt_f+1]


        pub_msg.header.seq = 0
        pub_msg.header.stamp = rospy.Time.now()
        pub_msg.goal_id.stamp = rospy.Time.now()
        pub_msg.goal_id.id = ''
        print pub_msg
        pub.publish(pub_msg)
        rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


























#if __name__ == '__main__':
#    rospy.init_node('publish_traj_from_bag')
#    rate = rospy.Rate(1) # 10hz
#    pub_msg = FollowJointTrajectoryActionGoal()
#    pub = rospy.Publisher('/arm_controller/follow_joint_trajectory/goal', FollowJointTrajectoryActionGoal, queue_size=10)
#    rate.sleep()
#    try:
#        if(len(sys.argv) < 2):
#            print("Err: specify bag as command line arg")
#            sys.exit()
#        bag = rosbag.Bag(sys.argv[1], 'r')
#        print "\n=========>  publish topic:/arm_controller/follow_joint_trajectory/goal "
#        for topic, msg, t in bag.read_messages(topics=["/arm_controller/follow_joint_trajectory/goal"]):
##            positions, velocities, accelerations, times = get_FJTAGoal_points(msg)
##            times += t.secs +  t.nsecs/1e9
#            pub_msg = msg
#        pub_msg.header.seq = 0
#        pub_msg.header.stamp = rospy.Time.now()
#        pub_msg.goal_id.stamp = rospy.Time.now()
#        pub_msg.goal_id.id = ''
#        print pub_msg
#        pub.publish(pub_msg)
#        rate.sleep()
#        rospy.spin()
#    except rospy.ROSInterruptException:
#        pass













