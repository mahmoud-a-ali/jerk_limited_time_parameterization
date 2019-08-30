/**
\file   quinticsplinetrajectory.h
\brief  definition of QuinticSplineRobotTrajectory Class
 * this class is used to genrate limited jerk time parameteriztion for group of joints(robot)
 * default controller/interpolation frequency: 125.
\author  Mahmoud Ali
\date    30/8/2019
*/



#ifndef QUINTICSPLINEROBOTTRAJECTORY_H
#define QUINTICSPLINEROBOTTRAJECTORY_H
#include<vector>


#include"quinticsplinetrajectory.h"
#include<trajectory_msgs/JointTrajectory.h>


class QuinticSplineRobotTrajectory
{
public:
private:
    int n_points_=0, n_segs_= 0, n_joints_=0;
    double max_pos_=0, max_vel_=0, max_acc_=0, max_jrk_=0, t_start_=0;
    std::vector<double> init_vel_,  init_acc_,  final_vel_,  final_acc_;

    std::vector<std::vector<double>> waypoints_pos_, waypoints_vel_, waypoints_acc_, times_, durations_;
    std::vector<double>sync_times_, sync_duration_;

    std::vector<QuinticSplineTrajectory> joint_trajectories_;


public:
  QuinticSplineRobotTrajectory();
  QuinticSplineRobotTrajectory(std::vector<std::vector<double> > waypoints_pos){
      std::cout<<"case 1 ..." <<std::endl;
      init( waypoints_pos, init_vel_,  init_acc_,  final_vel_,  final_acc_, t_start_);
  }
  QuinticSplineRobotTrajectory(std::vector<std::vector<double> > waypoints_pos, std::vector<double> init_vel, std::vector<double> init_acc, std::vector<double> final_vel, std::vector<double> final_acc){
      std::cout<<"case 2 ..." <<std::endl;
      init( waypoints_pos,  init_vel,  init_acc,  final_vel,  final_acc, t_start_);
  }
  QuinticSplineRobotTrajectory(std::vector<std::vector<double> > waypoints_pos, std::vector<double> init_vel, std::vector<double> init_acc, std::vector<double> final_vel, std::vector<double> final_acc, double start_time){
      std::cout<<"case 3 ..." <<std::endl;
      init( waypoints_pos,  init_vel,  init_acc,  final_vel,  final_acc, start_time);
  }
  QuinticSplineRobotTrajectory(trajectory_msgs::JointTrajectory traj_msg){
      std::cout<<"case 4 ..." <<std::endl;
      init_from_trajectory_msg(traj_msg);
  }


void set_absolute_limits(double max_pos, double max_vel, double max_acc, double max_jrk);

void init_from_trajectory_msg(trajectory_msgs::JointTrajectory traj_msg);
void init(std::vector<std::vector<double> > waypoints_pos, std::vector<double> init_vel, std::vector<double> init_acc, std::vector<double> final_vel, std::vector<double> final_acc, double start_time);

std::vector<std::vector<double>> update_waypoints_times();
std::vector<std::vector<double>> update_waypoints_times(double max_pos, double max_vel, double max_acc, double max_jrk);


void calculate_sync_vel_acc();
void calculate_sync_vel_acc(std::vector<double> sync_duration);
void update_jointTrajectory_msg( trajectory_msgs::JointTrajectory &msg);

void print_attributes();
void print_times();
void print_durations();
void print_waypoints_pos();
void print_waypoints_vel();
void print_waypoints_acc();


std::vector<std::vector<double>> get_calculated_durations(){ return durations_;}
std::vector<std::vector<double>> get_calculated_times(){ return times_;}
std::vector<std::vector<double>> get_calculated_pos(){ return waypoints_pos_;}
std::vector<std::vector<double>> get_calculated_vel(){ return waypoints_vel_;}
std::vector<std::vector<double>> get_calculated_acc(){ return waypoints_acc_;}


static void generate_sync_durations( std::vector< std::vector<double> > traj_durations, std::vector<double> &sync_durations);
static void compute_durations_from_times( std::vector<double> traj_times, std::vector<double>  &traj_durations);
static void compute_times_from_durations(double t_start, std::vector<std::vector<double>> traj_durations, std::vector< std::vector<double> > &traj_times );
static void compute_durations_from_times(std::vector< std::vector<double> > traj_times, std::vector< std::vector<double> > &traj_durations);
static void compute_times_from_durations( double t_start, std::vector<double>  traj_durations,  std::vector<double>  &traj_times );

};

#endif // QUINTICSPLINEROBOTTRAJECTORY_H
