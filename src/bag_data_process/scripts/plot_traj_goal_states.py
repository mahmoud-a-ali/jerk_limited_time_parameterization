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


def get_FJTGoal_points(msg):
    waypoints = None
    waypoints = msg.goal.trajectory.joint_trajectory.points
    positions = np.array([np.array(wp.positions) for wp in waypoints])
    velocities = np.array([np.array(wp.velocities) for wp in waypoints])
    accelerations = np.array([np.array(wp.accelerations) for wp in waypoints])
    times = np.array([wp.time_from_start.to_sec() for wp in waypoints])
    return (positions, velocities, accelerations, times)



def get_FJTAGoal_points(msg):
    waypoints = None
    waypoints = msg.goal.trajectory.points
    positions = np.array([np.array(wp.positions) for wp in waypoints])
    velocities = np.array([np.array(wp.velocities) for wp in waypoints])
    accelerations = np.array([np.array(wp.accelerations) for wp in waypoints])
    times = np.array([wp.time_from_start.to_sec() for wp in waypoints])
    return (positions, velocities, accelerations, times)


def get_JointState_points(msg):
    positions = [wp_pos for wp_pos in msg.position]
    velocities = [wp_vel for wp_vel in msg.velocity]
    return (positions, velocities)


def get_JointControllerState_points(msg):
    desired_pos = [wp_pos for wp_pos in msg.desired.positions]
    desired_vel = [wp_vel for wp_vel in msg.desired.velocities]
    actual_pos = [wp_pos for wp_pos in msg.actual.positions]
    actual_vel = [wp_vel for wp_vel in msg.actual.velocities]
    return (desired_pos, desired_vel, actual_pos, actual_vel)

def calc_derivative_from_sequence(sequence, times):
    derivatives = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    for point_i in range(len(sequence)-1):
        if times[point_i+1] - times[point_i] == 0:
            print("time delta is 0! skipping derivative")
            derivative = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        else:
            derivative = np.array([])
            for joint_i in range(len(sequence[point_i])):
                derivative = np.append(derivative, (sequence[point_i+1][joint_i] - sequence[point_i][joint_i]) / (times[point_i+1] - times[point_i]))
        derivatives = np.vstack((derivatives, derivative))

    return derivatives


if(len(sys.argv) < 2):
    print("Err: specify bag as command line arg")
    sys.exit()
bag = rosbag.Bag(sys.argv[1], 'r')


print "\n=========> topics "
#for topic, msg, t in bag.read_messages():
#    print topic
#for topic, msg, t in bag.read_messages(topics=["/arm_controller/follow_joint_trajectory/goal"]):
#    print topic


print "\n=========>  topic to plot:/arm_controller/follow_joint_trajectory/goal "
for topic, msg, t in bag.read_messages(topics=["/arm_controller/follow_joint_trajectory/goal"]):
    positions, velocities, accelerations, times = get_FJTAGoal_points(msg)
    times += t.secs +  t.nsecs/1e9 
    plt.figure(1)
    plt.subplot(2,1,1)
    plt.plot( times, positions)
    plt.plot( times, positions, "go")
    plt.legend( "123456")
    plt.title("goalpos")
    plt.grid()
    plt.subplot(2,1,2)
    plt.plot( times, velocities)
    plt.legend( "123456")
    plt.plot( times, velocities, "go")
    plt.title("goalvel")
    plt.grid()

    gl_pos_der1 = calc_derivative_from_sequence(positions, times)
    gl_pos_der2 = calc_derivative_from_sequence(gl_pos_der1, times)
    gl_pos_der3 = calc_derivative_from_sequence(gl_pos_der2, times)
    plt.figure(8)
    plt.subplot(3,1,1)
    plt.plot( times,  gl_pos_der1 )
    plt.legend( "123456")
    plt.grid()
    plt.title("gl_pos_der1")
    plt.subplot(3,1,2)
    plt.plot( times,  gl_pos_der2 )
    plt.legend( "123456")
    plt.grid()
    plt.title("gl_pos_der2")
    plt.subplot(3,1,3)
    plt.plot( times,  gl_pos_der3 )
    plt.legend( "123456")
    plt.grid()
    plt.title("gl_pos_der3")

    plt.figure(3)
    plt.subplot(2,1,1)
    plt.plot( times, positions, "go")
    plt.title("goal_js_pos")
    plt.grid()

    plt.subplot(2,1,2)
    plt.plot( times, velocities, "go")
    plt.title("goal_js_vel")
    plt.grid()


#    plt.show()



print "\n=========>  topic to plot: /joint_states "
pos_state = []
vel_state = []
t_js= []
for topic, msg, t in bag.read_messages(topics=["/joint_states"]):
    print "t: {}".format(t)
    print "t_stamp: {} ".format(msg.header.stamp)    
    pos, vel = get_JointState_points(msg)
    pos_state.insert(  len(pos_state), pos)
    vel_state.insert(  len(vel_state), vel)
    t_js.insert(  len(t_js), t.secs + t.nsecs/1e9)


plt.figure(2)
plt.subplot(2,1,1)
plt.plot(   t_js, pos_state)
plt.legend( "123456")
plt.grid()
plt.title("JS_pos")
plt.subplot(2,1,2)
plt.plot( t_js,  vel_state)
plt.legend( "123456")
plt.grid()
plt.title("JS_vel")


plt.figure(3)
plt.subplot(2,1,1)
plt.legend( "123456")
plt.plot(   t_js, pos_state)
plt.grid()

plt.subplot(2,1,2)
plt.legend( "123456")
plt.plot( t_js,  vel_state)
plt.grid()


print "\n=========>  topic to plot: /arm_controller/state"
t_cs = []
des_pos_ = []
des_vel_ = []
act_pos_ = []
act_vel_ = []
for topic, msg, t in bag.read_messages(topics=["/arm_controller/state"]):
    des_pos, des_vel, act_pos, act_vel = get_JointControllerState_points(msg)
    des_pos_.insert(  len(des_pos_), des_pos)
    des_vel_.insert(  len(des_vel_), des_vel)
    act_pos_.insert(  len(act_pos_), act_pos)
    act_vel_.insert(  len(act_vel_), act_vel)
    t_cs.insert( len(t_cs), t.secs +  t.nsecs/1e9)

plt.figure(4)
plt.subplot(2,1,1)
plt.plot( t_cs, des_pos_)
plt.legend( "123456")
plt.title("des_pos")
plt.grid()
plt.subplot(2,1,2)
plt.plot( t_cs, des_vel_)
plt.legend( "123456")
plt.title("des_vel")
plt.grid()


plt.figure(5)
plt.subplot(2,1,1)
plt.plot( t_cs, act_pos_)
plt.legend( "123456")
plt.title("act_pos")
plt.grid()
plt.subplot(2,1,2)
plt.plot( t_cs, act_vel_)
plt.legend( "123456")
plt.title("act_vel")
plt.grid()



plt.figure(6)
plt.subplot(2,1,1)
plt.plot( t_cs, act_pos_, "r*")
plt.legend( "123456")
plt.plot( t_cs, des_pos_ )
plt.title("act_des_pos")
plt.grid()

plt.subplot(2,1,2)
plt.plot( t_cs, act_vel_, "r*")
plt.legend( "123456")
plt.plot( t_cs, des_vel_ )
plt.title("act_des_vel")
plt.grid()


des_pos_der1 = calc_derivative_from_sequence(des_pos_, t_cs)
des_pos_der2 = calc_derivative_from_sequence(des_pos_der1, t_cs)
des_pos_der3 = calc_derivative_from_sequence(des_pos_der2, t_cs)
plt.figure(7)
plt.subplot(3,1,1)
plt.plot( t_cs,  des_pos_der1 )
plt.legend( "123456")
plt.grid()
plt.title("des_pos_der1")
plt.subplot(3,1,2)
plt.plot( t_cs,  des_pos_der2 )
plt.legend( "123456")
plt.grid()
plt.title("des_pos_der2")
plt.subplot(3,1,3)
plt.plot( t_cs,  des_pos_der3 )
plt.legend( "123456")
plt.grid()
plt.title("des_pos_der3")

#    plt.show()

bag.close()

print "already plotted .."
plt.show()





#for r in pos_state:
#    print  r[0]

#for t in t_js:
#    print  t

#for idx in range(0, len(t_js)):
#    print  "t, pos:  {}    {}".format(t_js[idx], pos_state[idx][0])



















