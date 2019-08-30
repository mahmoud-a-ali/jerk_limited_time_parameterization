/**
\file   quinticsplinetrajectory.cpp
\brief  definition of QuinticSplineRobotTrajectory Class
 * this class is used to genrate limited jerk time parameteriztion for group of joints(robot)
 * default controller/interpolation frequency: 125.
\author  Mahmoud Ali
\date    30/8/2019
*/

#include "quinticsplinerobottrajectory.h"

QuinticSplineRobotTrajectory::QuinticSplineRobotTrajectory()
{

}


// ========================== init from a message =============================
void QuinticSplineRobotTrajectory::init_from_trajectory_msg( trajectory_msgs::JointTrajectory msg){ //}, double t_start ){
    std::cout<<">>>> init QuinticSplineRobotTrajectory_from_traj_msg: "<<std::endl;

    int n_jts = msg.joint_names.size();
    int n_pts = msg.points.size();
    std::cout<<"#n_jts, n_pts: "<< n_jts << ",  "<< n_pts <<std::endl;
    // extract Positions, Velocities ,Accelerations, Times from that trajectory and store them in std::vectors
    std::vector< std::vector<double> > Pos_jt_wpt, Vel_jt_wpt, ACC_jt_wpt, Times_jt_wpt, durations;
    Pos_jt_wpt.resize(n_jts);
    Vel_jt_wpt.resize(n_jts);
    ACC_jt_wpt.resize(n_jts);
    Times_jt_wpt.resize(n_jts);
    durations.resize(n_jts);
    for(int jt=0; jt<n_jts; jt++){
        for(int pt=0; pt<n_pts; pt++){
            Pos_jt_wpt[jt].push_back(  msg.points[pt].positions[jt] ); //57.2958*
            Vel_jt_wpt[jt].push_back(  0*msg.points[pt].velocities[jt] ); //57.2958* //not used any more
            ACC_jt_wpt[jt].push_back(  0*msg.points[pt].accelerations[jt] ); //57.2958* //not used any more
            Times_jt_wpt[jt].push_back( msg.points[pt].time_from_start.toSec()); //already converted in stored traj
        }
    }
    double start_time =Times_jt_wpt[0][0];
    std::vector<double> init_vel, init_acc, final_vel, final_acc;
    init_vel.resize(n_jts);
    init_acc.resize(n_jts);
    final_vel.resize(n_jts);
    final_acc.resize(n_jts);
    for (int jt; jt< n_jts; jt++) {
        init_vel[jt]=Vel_jt_wpt[jt][0];
        init_acc[jt]=ACC_jt_wpt[jt][0];
        final_vel[jt]=Vel_jt_wpt[jt][n_pts-1];
        final_acc[jt]=ACC_jt_wpt[jt][n_pts-1];

    }
//    std::cout<<">>>> init QuinticSplineRobotTrajectory_from_msg done: call init now "<<std::endl;
    // init traj using waypoints, initial and final vel,acc
    init(Pos_jt_wpt, init_vel,  init_acc,  final_vel, final_acc, start_time);
}




// ==================================== init QuinticSplineRobotTrajectory ============================
void QuinticSplineRobotTrajectory::init(std::vector<std::vector<double> > waypoints_pos, std::vector<double> init_vel, std::vector<double> init_acc, std::vector<double> final_vel, std::vector<double> final_acc, double start_time){
    if (waypoints_pos.empty() ) // || waypoints_vel.empty() || waypoints_acc.empty()  )
      throw(std::invalid_argument("QuinticSplineRobotTrajectory can't be constructed: waypoints_pos are empty "));
    else{
        waypoints_pos_ = waypoints_pos;
        n_joints_ = waypoints_pos.size();
        n_points_ = waypoints_pos[0].size();
        n_segs_ = n_points_ -1;
        waypoints_vel_.resize(n_joints_);
        waypoints_acc_.resize(n_joints_);
        //std::cout<<"initiate QuinticSplineRobotTrajectory: n_joints_ =  "<< n_joints_ <<"  && n_points_ = "<< n_points_ <<std::endl;
    }
    if (!waypoints_pos.empty() && !init_vel.empty() && (waypoints_pos.size() != init_vel.size() ) )
      throw(std::invalid_argument("QuinticSplineRobotTrajectory can't be constructed: positions and init velocities have different joints sizes"));
    if (!waypoints_pos.empty() && !init_acc.empty() && (waypoints_pos.size() != init_acc.size() ) )
      throw(std::invalid_argument("QuinticSplineRobotTrajectory can't be constructed: positions and init accelerations have different joints sizes"));
    if (!waypoints_pos.empty() && !final_vel.empty() && (waypoints_pos.size() != final_vel.size() ) )
      throw(std::invalid_argument("QuinticSplineRobotTrajectory can't be constructed: positions and final velocities have different joints sizes"));
    if (!waypoints_pos.empty() && !final_acc.empty() && (waypoints_pos.size() != final_acc.size() ) )
      throw(std::invalid_argument("QuinticSplineRobotTrajectory can't be constructed: positions and final accelerations have different joints sizes"));


    // get IC, and FC, if not provided consider all of them zeros
    if ( !init_vel.empty() && !init_acc.empty() && !final_vel.empty() && !final_acc.empty() ){
        //std::cout<<"-- not empty " <<std::endl;
        init_vel_ = init_vel;  final_vel_= final_vel;
        init_acc_= init_acc;   final_acc_= final_acc;
    }else{
        init_vel_.resize(n_joints_);  final_vel_.resize(n_joints_);
        init_acc_.resize(n_joints_);  final_acc_.resize(n_joints_);
        for (int jt=0; jt< n_joints_; jt++){
          init_vel_[jt] =0;
          final_vel_[jt]=0;
          init_acc_[jt] =0;
          final_acc_[jt]=0;
        }
    }
    t_start_ = start_time;
    times_.resize(n_joints_);
    durations_.resize(n_joints_);

    sync_times_.resize( n_points_);
    sync_duration_.resize( n_segs_);

    // just initilization
    sync_times_[0]= t_start_;
    for (int pt=0; pt< n_segs_; pt++)
        sync_duration_[pt] =0;

    for (int pt=0; pt< n_points_-1; pt++)
        sync_times_[pt+1] = sync_times_[pt] + sync_duration_[pt];

//    std::cout<<"-- initiate individual trajcctories " <<std::endl;
    joint_trajectories_.resize( n_joints_);
    for (int jt=0; jt< n_joints_; jt++)
        joint_trajectories_[jt].init(waypoints_pos[jt], init_vel_[jt], init_acc_[jt], final_vel_[jt], final_acc_[jt]);//, t_start_);
//    std::cout<<">>>> init done: "<<std::endl;
}




//=================set limit==============================
void QuinticSplineRobotTrajectory::set_absolute_limits(double max_pos, double max_vel, double max_acc, double max_jrk){
    if(max_pos < 0 || max_vel < 0 || max_acc < 0|| max_jrk < 0)
        throw(std::invalid_argument("absolute limits should be positive"));
    max_pos_ = max_pos;
    max_vel_ = max_vel;
    max_acc_ = max_acc;
    max_jrk_ = max_jrk;
    for (int jt=0; jt< n_joints_; jt++){
//        std::cout<<"set_absolute_limits: traj_jt= "<< jt <<std::endl;
        joint_trajectories_[jt].set_absolute_limits(max_pos_, max_vel_, max_acc_, max_jrk_);
    }
//    std::cout<<"set_absolute_limits: done" <<std::endl;
}



//================== update times and velocities for traj_message==================
void QuinticSplineRobotTrajectory::update_jointTrajectory_msg( trajectory_msgs::JointTrajectory &msg){
    if(max_pos_ <= 0 || max_vel_ <= 0 || max_acc_ <= 0|| max_jrk_ <= 0)
        throw(std::invalid_argument("segment can't be constructed: absolute limits should be positive"));
    for (int pt = 0; pt < n_points_; ++pt) {
        for (int jt=0; jt<n_joints_; jt++) {
            msg.points[pt].positions[jt]= waypoints_pos_[jt][pt];//57.2958;
            msg.points[pt].velocities[jt] = waypoints_vel_[jt][pt];//57.2958;
            msg.points[pt].accelerations[jt] = waypoints_acc_[jt][pt];//57.2958;
            msg.points[pt].time_from_start = ros::Duration(times_[jt][pt]);
        }
    }


}




//=============update each joint trajectory seperately=================
std::vector<std::vector<double>> QuinticSplineRobotTrajectory::update_waypoints_times(){
    if( max_pos_>0 && max_vel_>0 && max_acc_>0 && max_jrk_>0)
        return update_waypoints_times(max_pos_, max_vel_, max_acc_, max_jrk_);
    else
        throw( std::invalid_argument("absolute limits (pos, vel, acc, jrk) should be positive and non-zero values") );
}

//=============update each joint trajectory seperately, 2nd footprint=================
std::vector<std::vector<double>> QuinticSplineRobotTrajectory::update_waypoints_times(double max_pos, double max_vel, double max_acc, double max_jrk){
//    std::cout<<"\n>>>>>>>>>>>>update_waypoints_times" << std::endl;
    set_absolute_limits(max_pos, max_vel, max_acc, max_jrk);
    for (int jt=0; jt< n_joints_; jt++) {
//        std::cout<<"\n\n ====================== update_waypoints_times: joint_traj: "<< jt << " =======================" << std::endl;
        times_[jt] = joint_trajectories_[jt].update_waypoints_times();
//        std::cout<<"\n jt_"<<  jt<< "update times: "<<std::endl; // chcek how times are updated
//        for (int i=0; i< times_[jt].size(); i++)
//            std::cout<<"  "<< times_[jt][i];
//        std::cout<<std::endl;
        compute_durations_from_times(times_[jt], durations_[jt]);
        waypoints_vel_[jt] = joint_trajectories_[jt].get_calculated_velocities();
        waypoints_acc_[jt] = joint_trajectories_[jt].get_calculated_accelerations();

    }
    return times_;
}



//================= calculate vel, acc for the sync_duration=======================
// it should overwrite vel, acc, times for all joints
void QuinticSplineRobotTrajectory::calculate_sync_vel_acc(){
    update_waypoints_times();
    print_durations();
    generate_sync_durations(durations_, sync_duration_);
    calculate_sync_vel_acc(sync_duration_);
}
//======another  version for external use  ======
void QuinticSplineRobotTrajectory::calculate_sync_vel_acc(std::vector<double> sync_duration){
//    std::cout<<"\n_____________ solve for sync vel acc _____________"<<std::endl;
    for (int jt=0; jt< n_joints_; jt++){
//        std::cout<<"\n\n............. jt: "<< jt<< " ................"<<std::endl;
        joint_trajectories_[jt].solve_for_vel_acc(sync_duration);
        durations_[jt] = joint_trajectories_[jt].get_calculated_durations();
        times_[jt] = joint_trajectories_[jt].get_calculated_times();
//        waypoints_pos_[jt] = joint_trajectories_[jt].get_calculated_positions();
        waypoints_vel_[jt] = joint_trajectories_[jt].get_calculated_velocities();
        waypoints_acc_[jt] = joint_trajectories_[jt].get_calculated_accelerations();
    }
}



void QuinticSplineRobotTrajectory::generate_sync_durations( std::vector< std::vector<double> > traj_durations, std::vector<double> &sync_durations){
    int n_jts= traj_durations.size();
    int n_pts= traj_durations[0].size() +1;
    sync_durations.resize(n_pts-1);

    //takes max durations for each segment over different joints/traajectories, create synchronised durations
    for(int seg=0; seg<n_pts-1 ; seg++){
       sync_durations[seg] = traj_durations[0][seg];
       for(int jt=1; jt<n_jts ; jt++){
           if( sync_durations[seg] < traj_durations[jt][seg]  )
               sync_durations[seg]= traj_durations[jt][seg] ;
       }
    }
}



void QuinticSplineRobotTrajectory::compute_durations_from_times( std::vector<double> traj_times, std::vector<double>  &traj_durations){
    //create trajectories durations from times
    int n_pts=traj_times.size();
    traj_durations.resize( n_pts -1);
        for(int seg=0; seg<n_pts-1; seg++)  //n_segs = n_pts-1
            traj_durations[seg] = traj_times[seg+1] - traj_times[seg];
}


void QuinticSplineRobotTrajectory::compute_times_from_durations( double t_start, std::vector<double>  traj_durations,  std::vector<double>  &traj_times ){
    //create trajectories times from durations
    int n_pts=traj_durations.size()+1;
    traj_times.resize( n_pts );
    traj_times[0]= t_start;

    for (int pt=0; pt< n_pts-1; pt++)
            traj_times[pt+1] = traj_times[pt] + traj_durations[pt]; // t= last_time + duration of that segment

}


void QuinticSplineRobotTrajectory::compute_durations_from_times(std::vector< std::vector<double> > traj_times, std::vector< std::vector<double> > &traj_durations){
    //create trajectories durations from times
    int n_jts=traj_times.size();
    int n_pts=traj_times[0].size();
    traj_durations.resize( n_jts );

    for(int jt=0; jt<n_jts; jt++)
        for(int seg=0; seg<n_pts-1; seg++)  //n_segs = n_pts-1
            traj_durations[jt].push_back( traj_times[jt][seg+1] - traj_times[jt][seg]);
}


void QuinticSplineRobotTrajectory::compute_times_from_durations(double t_start, std::vector<std::vector<double>> traj_durations, std::vector< std::vector<double> > &traj_times ){
    //create trajectories times from durations
    int n_jts=traj_durations.size();
    int n_pts=traj_durations[0].size()+1;
    traj_times.resize( n_jts );

    for(int jt=0; jt<n_jts; jt++){
        traj_times[jt][0]= t_start;
        for (int pt=0; pt< n_pts-1; pt++)
            traj_times[jt][pt+1] = traj_times[jt][pt] + traj_durations[jt][pt]; // t= last_time + duration of that segment
     }
}



void QuinticSplineRobotTrajectory::print_attributes(){
    std::cout<<"============================ attributes QuinticSplineRobotTrajectory ============================"<<std::endl;
    std::cout<<"###limits: \n" <<"n_pts: " <<n_points_ <<",  n_jts: " <<n_joints_<<", n_seg: " <<n_segs_<<std::endl;
    std::cout<<"###limits: \n" <<"pos: " <<max_pos_ <<",  vel: " <<max_vel_<<", acc: " <<max_acc_<<",  jrk: " <<max_jrk_<<std::endl;
    for (int jt=0; jt< n_joints_; jt++) {
        std::cout<<"--------------------------- attributes of joint "<< jt<<"-----------------------------"<<std::endl;
        joint_trajectories_[jt].print_attributes();
    }

}


void QuinticSplineRobotTrajectory::print_times(){
std::cout<<"\n print_times: jts, pts: "<< n_joints_<<",  "<<n_points_<< std::endl;
    for (int jt = 0; jt < times_.size(); ++jt){
        std::cout<<"jt_"<<jt<<" :";
        for (int pt = 0; pt < times_[0].size(); ++pt)
            std::cout<<"  "<< times_[jt][pt];
        std::cout<<std::endl;
    }
}


void QuinticSplineRobotTrajectory::print_durations(){
std::cout<<"\n print_durations: jts, pts: "<< n_joints_<<",  "<<n_points_<< std::endl;
    for (int jt = 0; jt < durations_.size(); ++jt){
        std::cout<<"jt_"<<jt<<" :";
        for (int pt = 0; pt < durations_[0].size(); ++pt)
            std::cout<<"  "<< durations_[jt][pt];
        std::cout<<std::endl;
    }
}


void QuinticSplineRobotTrajectory::print_waypoints_pos(){
std::cout<<"\n print_waypoints_pos: jts, pts: "<< n_joints_<<",  "<<n_points_<< std::endl;
    for (int jt = 0; jt < waypoints_pos_.size(); ++jt){
        std::cout<<"jt_"<<jt<<" :";
        for (int pt = 0; pt <waypoints_pos_[0].size(); ++pt)
            std::cout<<"  "<<waypoints_pos_[jt][pt];
        std::cout<<std::endl;
    }
}



void QuinticSplineRobotTrajectory::print_waypoints_vel(){
std::cout<<"\n print_waypoints_vel: jts, pts: "<< n_joints_<<",  "<<n_points_<< std::endl;
    for (int jt = 0; jt < waypoints_vel_.size(); ++jt){
        std::cout<<"jt_"<<jt<<" :";
        for (int pt = 0; pt <waypoints_vel_[0].size(); ++pt)
            std::cout<<"  "<<waypoints_vel_[jt][pt];
        std::cout<<std::endl;
    }
}


void QuinticSplineRobotTrajectory::print_waypoints_acc(){
std::cout<<"\n print_waypoints_acc: jts, pts: "<< n_joints_<<",  "<<n_points_<< std::endl;
    for (int jt = 0; jt < waypoints_acc_.size(); ++jt){
        std::cout<<"jt_"<<jt<<" :";
        for (int pt = 0; pt <waypoints_acc_[0].size(); ++pt)
            std::cout<<"  "<<waypoints_acc_[jt][pt];
        std::cout<<std::endl;
    }
}





