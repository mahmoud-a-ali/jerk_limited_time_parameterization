/**
\file   QuinticSplineTrajectory.h
\brief  definition of QuinticSplineTrajectory Class
 * this class is used to genrate limited jerk time parameteriztion for single joint
 * default controller/interpolation frequency: 125.
\author  Mahmoud Ali
\date    30/8/2019
*/


#ifndef QUINTICSPLINETRAJECTORY_H
#define QUINTICSPLINETRAJECTORY_H

#include "bezierquinticsegment.h"
#include <Eigen/Dense>
using namespace Eigen;
using Eigen::MatrixXd;


class QuinticSplineTrajectory
{

private:
    int n_points_=0, n_segs_= 0, n_eqs_=0;
    double max_pos_=0, max_vel_=0, max_acc_=0, max_jrk_=0, t_start_=0;
    double init_vel_=0,  init_acc_=0,  final_vel_=0,  final_acc_=0;
    std::vector<double> positions_, times_, durations_;
    std::vector<double> cal_positions_, cal_velocities_, cal_accelerations_, cal_jerks_;

    VectorXd dur_vec_, B_vec_, X_vec_;
    MatrixXd A_mtrx_ ;
    std::vector<std::vector<double>> cof_mtrx_;


public:
  QuinticSplineTrajectory();
  QuinticSplineTrajectory(std::vector<double> waypoints_pos){
      init( waypoints_pos, 0,0,0,0); // IC=FC=0 for vel and acc
  }
  QuinticSplineTrajectory(std::vector<double> waypoints_pos, double init_vel, double init_acc, double final_vel, double final_acc){
      init( waypoints_pos,  init_vel,  init_acc,  final_vel,  final_acc);
  }

void init(std::vector<double> waypoints_pos, double init_vel, double init_acc, double final_vel, double final_acc);
void set_absolute_limits(double max_pos, double max_vel, double max_acc, double max_jrk);

void set_init_duration();
void set_durations( std::vector<double> durations);

static void generate_coef_matrix_c1(const int idx, const VectorXd &h, MatrixXd &C1);
void solve_for_quintic_coef();
void solve_for_vel_acc_times();
void solve_for_vel_acc_times(double max_pos, double max_vel, double max_acc, double max_jrk);


int find_seg_number (const double &tg, const std::vector<double> &T_vec);
int sample (const std::vector<double>& T_vec, const double& tg,const std::vector<std::vector<double>> &cof,  std::vector<double> & state);


std::vector<int> check_limits();
std::vector<int> check_monotonic();
bool check_behavoir(std::vector<int> &unlimited_seg_id, std::vector<int> &nonmono_seg_id);

std::vector<double>  update_waypoints_times();
void solve_for_vel_acc( std::vector<double> sync_dur );


void print_attributes();


std::vector<double> get_calculated_durations(){return durations_;}
std::vector<double> get_calculated_times(){return times_;}
std::vector<double> get_calculated_positions(){return cal_positions_;}
std::vector<double> get_calculated_velocities(){return cal_velocities_;}
std::vector<double> get_calculated_accelerations(){return cal_accelerations_;}
std::vector<double> get_calculated_jerks(){return cal_jerks_;}
std::vector< std::vector<double> > get_coef_mtrx(){return cof_mtrx_;}
};

#endif // QUINTICSPLINETRAJECTORY_H
