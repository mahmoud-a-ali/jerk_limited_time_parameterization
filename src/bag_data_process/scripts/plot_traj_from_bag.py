#!/usr/bin/env python
import numpy as np
import rosbag
import sys

import rospy
import numpy as np
import yaml
import os
#import matplotlib
from matplotlib import pyplot as plt


from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryActionGoal
from control_msgs.msg import JointTrajectoryGoal
from rospy_message_converter import message_converter
from sensor_msgs.msg import JointState

def get_waypoints(bag_name):
    bag = rosbag.Bag(bag_name, 'r')
    waypoints = None
    for topic, msg, t in bag.read_messages():
        print("Number of waypoints: ", len(msg.goal.trajectory.joint_trajectory.points))
        waypoints = msg.goal.trajectory.joint_trajectory.points
    bag.close()

    positions = np.array([np.array(wp.positions) for wp in waypoints])
    velocities = np.array([np.array(wp.velocities) for wp in waypoints])
    accelerations = np.array([np.array(wp.accelerations) for wp in waypoints])
    times = np.array([wp.time_from_start.to_sec() for wp in waypoints])
    return (positions, velocities, accelerations, times)


def get_FJTGoal_points(msg):
    waypoints = None
    waypoints = msg.goal.trajectory.joint_trajectory.points
    positions = np.array([np.array(wp.positions) for wp in waypoints])
    velocities = np.array([np.array(wp.velocities) for wp in waypoints])
    accelerations = np.array([np.array(wp.accelerations) for wp in waypoints])
    times = np.array([wp.time_from_start.to_sec() for wp in waypoints])
    return (positions, velocities, accelerations, times)




if(len(sys.argv) < 2):
    print("Err: specify bag as command line arg")
    sys.exit()
bag = rosbag.Bag(sys.argv[1], 'r')



for topic, msg, t in bag.read_messages():
    print topic

for topic, msg, t in bag.read_messages(topics=["/arm_controller/follow_joint_trajectory/goal"]):
    print topic



t_js = np.array([])
for topic, msg, t in bag.read_messages(topics=["/execute_trajectory/goal"]):
#    print "############ topics: {}".format(topic )
    print "############ msg   : {}".format(msg)
    (positions, velocities, accelerations, times) = get_FJTGoal_points(msg)
#    ts =t.secs
#    t_js = np.append( t_js, ts)

#    print len(t_js)
#    print len(positions)
    plt.figure(1)
    plt.subplot(2,2,1)
    plt.plot( times, positions)
    plt.plot( times, positions, "r*")


    plt.title("pos")

    plt.subplot(2,2,2)
    plt.plot( times, velocities)
    plt.title("vel")

    plt.subplot(2,2,3)
    plt.plot( times, accelerations)
    plt.title("acc")

#    plt.show()


pos = np.array([])
vel = np.array([])
acc = np.array([])
t_js = np.array([])
for topic, msg, t in bag.read_messages(topics=["/joint_states"]):
#    print "############ topics: {}".format(topic )
    print "############ msg   : "#{}".format(msg)
    pos = np.append( pos, msg.position[0])
    vel = np.append( vel, msg.velocity[0])
#    acc = np.append( pos, msg.acceleration[0])
#    pos = np.concatenate( (pos, msg.position[0]) )
    ts =t.secs
    t_js = np.append( t_js, ts)
    print msg.position
    print msg.header.stamp

print "\n len:"
print len(t_js)
print len(pos)
print t_js[0]
print pos[0]

plt.figure()
plt.subplot(2,2,1)
plt.plot(    pos)
#plt.plot(   pos, "r*")
plt.title("pos")

plt.subplot(2,2,2)
plt.plot(  vel)
plt.title("vel")

#plt.subplot(2,2,3)
#plt.plot(  acc)
#plt.title("acc")

#plt.show()





#(positions, velocities, accelerations, times) = get_waypoints(sys.argv[1])
#plt.plot(times, positions)
#print "already plotted .."
#plt.show()




#bag = rosbag.Bag(sys.argv[1], 'r')
#for topic, msg, t in bag.read_messages():
#    print len(topic)
#    print topic

#pos = np.array()
#for topic, msg, t in bag.read_messages():
#    if(topic == "/execute_trajectory/goal"):
##        print msg
#        for pt in msg.goal.trajectory.joint_trajectory.points:
#            print "\n here is  msg:"
#            print pt
#            pos = pos.append(pt.positions)
#print pos


#for topic, msg, t in bag.read_messages(topics=["/arm_controller/follow_joint_trajectory/goal"]):
#    print msg



#bag = rosbag.Bag('test.bag')
#for topic, msg, t in bag.read_messages(topics=['chatter', 'numbers']):
#    print msg
#bag.close()

#for topic, msg, t in bag.read_messages(topics=["/arm_controller/follow_joint_trajectory/goal"]):
#    print len(topic)


#for topic, msg, t in bag.read_messages():
#    if(topic == "/arm_controller/follow_joint_trajectory/goal"):
#        print topic

#for topic, msg, t in bag.read_messages(topics=["/arm_controller/follow_joint_trajectory/goal"]):
#    print msg



#bag.close()

#for topic, msg, t in bag.read_messages():
#    print("Number of waypoints: ", len(msg.goal.trajectory.joint_trajectory.points))
#    waypoints = msg.goal.trajectory.joint_trajectory.points
#bag.close()

#positions = np.array([np.array(wp.positions) for wp in waypoints])





print "already plotted .."


#(positions, velocities, accelerations, times) = get_waypoints(sys.argv[1])
#plt.plot(positions[0])
#print "already plotted .."
#for pt in positions:
#    print pt


plt.show()
