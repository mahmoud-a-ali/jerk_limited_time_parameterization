/**
\file   bezieer_from_msg_online_test.h
\brief  this node receives FollowJointTrajectoryActionGoal from moveit
 * it uses QuinticSplineRobotTrajectory class to update times, vel, acc
 * and then it uses bezier_robot_trajectory to interpolate the new trajectory
 * default controller/interpolation frequency: 125.
\author  Mahmoud Ali
*/


#include <ros/ros.h>
#include "bezier_trajectory.h" // using thi one for interpolation, it interpolate trajectory as the bezier version of the joint_trajectoy_controller does
#include "quinticsplinerobottrajectory.h"

#include<trajectory_msgs/JointTrajectory.h>
#include<control_msgs/FollowJointTrajectoryActionGoal.h>


#include <python2.7/Python.h>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

// limits (unit: rad, rad/sec, rad/sec^2 .. etc)
const double max_pos=180*0.0174533, max_vel= 130*0.0174533, max_acc=250*0.0174533, max_jrk=1000*0.0174533, frq=125; // pos, vel, acc, jrk max limits (*0.0174533  and *57.2958)
control_msgs::FollowJointTrajectoryActionGoal traj_goal;
// to plot old and new values of pos, vel, acc before and after retiming occurs
std::vector<std::vector<double>> old_times, old_pos, old_vel, old_acc;
std::vector<std::vector<double>> new_times, new_pos, new_vel, new_acc;
bool traj_received = false; // boolean variable to check for first msg



// to print 2d vector in nice way
void print_2dVec(std::vector<std::vector<double>> Vvec, std::string vec_name){
    std::cout<<"\n" <<vec_name<<": "<< Vvec.size()<<",  "<<Vvec[0].size()<< "  (main)\n";
    for (int jt = 0; jt < Vvec.size(); ++jt){
        std::cout<<"jt_"<<jt<<" :";
        for (int pt = 0; pt < Vvec[0].size(); ++pt)
            std::cout<<"  "<< Vvec[jt][pt];
        std::cout<<std::endl;
    }

}




// call back with no retiming occurs
void goal_callback(const control_msgs::FollowJointTrajectoryActionGoal &msg){
    traj_received = true;
    traj_goal = msg;
    ROS_INFO(" goal received .... ");
}


// call back which performs retiming for the the trajectory
void updated_traj_callback( const  control_msgs::FollowJointTrajectoryActionGoal &updated_msg){
    traj_received = true; // boolean variable to check for new msg
    // check number of joints and points in the coming msg
    int n_jts = updated_msg.goal.trajectory.joint_names.size();
    int n_pts = updated_msg.goal.trajectory.points.size()-2;

    // clear all variables that were used to plot last trajectories
    old_times.clear();  old_pos.clear();  old_vel.clear();  old_acc.clear();
    new_times.clear();  new_pos.clear();  new_acc.clear();  new_vel.clear();
    old_times.resize(n_jts); old_pos.resize(n_jts); old_vel.resize(n_jts); old_acc.resize(n_jts);
    new_times.resize(n_jts); new_pos.resize(n_jts); new_vel.resize(n_jts); new_acc.resize(n_jts);


    control_msgs::FollowJointTrajectoryActionGoal updated_goal;
    trajectory_msgs::JointTrajectory temp_msg = updated_msg.goal.trajectory;

    // instiniate an object of QuinticSplineRobotTrajectory to update times, vel, acc
    QuinticSplineRobotTrajectory rbt_traj(temp_msg);//updated_msg.goal.trajectory);
    rbt_traj.set_absolute_limits(max_pos, max_vel, max_acc, max_jrk);//(180, 130, 250, 1000);//(sm, vm, am, jm);
    rbt_traj.calculate_sync_vel_acc();

    // to check values
    rbt_traj.print_times();
    rbt_traj.print_durations();
    rbt_traj. print_waypoints_pos();
    rbt_traj.print_waypoints_vel();
    rbt_traj.print_waypoints_acc();


    rbt_traj.update_jointTrajectory_msg(temp_msg);
    updated_goal.goal.trajectory = temp_msg ;
    updated_goal.header =  updated_msg.header ;
    updated_goal.goal_id =  updated_msg.goal_id ;


    new_times = rbt_traj.get_calculated_times();
    new_pos = rbt_traj.get_calculated_pos();
    new_vel = rbt_traj.get_calculated_vel();
    new_acc = rbt_traj.get_calculated_acc();
    // to check values
//    print_2dVec( new_times, "new_times");
//    print_2dVec( new_pos, "new_pos");
//    print_2dVec( new_vel, "new_vel");
//    print_2dVec( new_acc, "new_acc");

//    std::cout<<"\n\n\n>>>>>>>>>>>>> old trajectory <<<<<<<<<<<<< ";
//    for (int jt=0; jt<n_jts; jt++) {
//        std::cout<<"\n#########joints: "<< jt<< " ###########"<<std::endl;
//        std::cout<<"positions     :  ";
//        for (int pt = 0; pt < n_pts; ++pt)
//            std::cout<<"  "<< updated_msg.goal.trajectory.points[pt].positions[jt];
//        std::cout<<"\nvelocities    :  ";
//        for (int pt = 0; pt < n_pts; ++pt)
//            std::cout<<"  "<< updated_msg.goal.trajectory.points[pt].velocities[jt];
//        std::cout<<"\naccelerations :  ";
//        for (int pt = 0; pt < n_pts; ++pt)
//            std::cout<<"  "<< updated_msg.goal.trajectory.points[pt].accelerations[jt];
//            std::cout<<"\ndurations     :  ";
//            for (int pt = 0; pt < n_pts; ++pt)
//                std::cout<<"  "<<  updated_msg.goal.trajectory.points[pt].time_from_start.toSec();
//            std::cout<<std::endl;
//    }

//    std::cout<<"\n\n\n>>>>>>>>>>>>> new trajectory <<<<<<<<<<<<< ";
//    for (int jt=0; jt<n_jts; jt++) {
//        std::cout<<"\n#########joints: "<< jt<< " ###########"<<std::endl;
//        std::cout<<"positions     :  ";
//        for (int pt = 0; pt < n_pts; ++pt)
//            std::cout<<"  "<< temp_msg.points[pt].positions[jt];
//        std::cout<<"\nvelocities    :  ";
//        for (int pt = 0; pt < n_pts; ++pt)
//            std::cout<<"  "<< temp_msg.points[pt].velocities[jt];
//        std::cout<<"\naccelerations :  ";
//        for (int pt = 0; pt < n_pts; ++pt)
//            std::cout<<"  "<< temp_msg.points[pt].accelerations[jt];
//        std::cout<<"\ndurations     :  ";
//        for (int pt = 0; pt < n_pts; ++pt)
//            std::cout<<"  "<<  temp_msg.points[pt].time_from_start.toSec();
//        std::cout<<std::endl;
//    }

    traj_goal = updated_goal; // this the msg will be passed to main function


    //storing old values to compare plotting
    for (int jt = 0; jt < n_jts; ++jt){
        for (int pt = 0; pt < n_pts; ++pt){
            old_times[jt].push_back( updated_msg.goal.trajectory.points[pt].time_from_start.toSec() );
            old_pos[jt].push_back( updated_msg.goal.trajectory.points[pt].positions[jt] );
            old_vel[jt].push_back( updated_msg.goal.trajectory.points[pt].velocities[jt] );
            old_acc[jt].push_back( updated_msg.goal.trajectory.points[pt].accelerations[jt] );
        }
    }
//    for (int jt=0; jt<n_jts; jt++) {
//        plt::figure(1+jt);
//        plt::subplot(2, 2, 1);
//        plt::named_plot( "pos",old_times[jt], old_pos[jt]);
//        plt::named_plot( "pos",old_times[jt], old_pos[jt], "r*"); plt::grid(true); plt::title("pos"); //plt::legend();
//        plt::subplot(2, 2, 2);
//        plt::named_plot( "vel",old_times[jt], old_vel[jt]);
//        plt::named_plot( "vel",old_times[jt], old_vel[jt], "r*"); plt::grid(true); plt::title("vel");
//        plt::subplot(2, 2, 3);
//        plt::named_plot( "acc",old_times[jt], old_acc[jt]); plt::grid(true); plt::title("acc");
//    }

//    for (int jt = 0; jt < n_jts; ++jt){
//        for (int pt = 0; pt < n_pts; ++pt){
//            new_times[jt].push_back( temp_msg.points[pt].time_from_start.toSec() );
//            new_pos[jt].push_back( temp_msg.points[pt].positions[jt] );
//            new_vel[jt].push_back( temp_msg.points[pt].velocities[jt] );
//            new_acc[jt].push_back( temp_msg.points[pt].accelerations[jt] );
//        }
//    }

    // here plots the pos, vel, acc of waypoints (no interpolation) for each joint seperatly
//    for (int jt=0; jt<n_jts; jt++) {
//        plt::figure(11+jt);
//        plt::subplot(2, 2, 1);
//        plt::named_plot( "pos",new_times[jt], new_pos[jt]);
//        plt::named_plot( "pos",new_times[jt], new_pos[jt], "r*"); plt::grid(true); plt::title("pos"); //plt::legend();
//        plt::subplot(2, 2, 2);
//         plt::named_plot( "vel",new_times[jt], new_vel[jt]);
//        plt::named_plot( "vel",new_times[jt], new_vel[jt], "r*"); plt::grid(true); plt::title("vel");
//        plt::subplot(2, 2, 3);
//        plt::named_plot( "acc",new_times[jt], new_acc[jt]); plt::grid(true); plt::title("acc");

//    }
//    plt::show();


}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "bezier_from_msg_online_test");
    ros::NodeHandle nh;

    ROS_INFO("initializing path .... ");
//    ros::Subscriber goal_sub = nh.subscribe( "/arm_controller/follow_joint_trajectory/goal", 1000, goal_callback );
    ros::Subscriber updated_goal_sub = nh.subscribe( "/arm_controller/follow_joint_trajectory/goal", 1000, updated_traj_callback );

    while( ros::ok() ){
        ros::spinOnce();
        if( !traj_received ) // no new msg
            continue;
        traj_received = false;
        ROS_INFO(" goal received in  main.... ");

        int n_jts = traj_goal.goal.trajectory.joint_names.size();
        int n_pts = traj_goal.goal.trajectory.points.size();

        // instantiate an object of the bezier_robot_trajectory will be used here for interpolation
        bezier_robot_trajectory rbt_traj(traj_goal.goal.trajectory);
        rbt_traj.set_absolute_limits(max_pos, max_vel, max_acc, max_jrk);
        // get times for plotting,
        std::vector<double> sync_times;
        for (int pt=0; pt<n_pts; pt++)
            sync_times.push_back( traj_goal.goal.trajectory.points[pt].time_from_start.toSec() );


        // use sample function to sample all joint trajectories and plot them
        std::vector<double> sampled_time, pos, vel, acc, jrk ;
        std::vector<std::vector<double>>  POS, VEL,ACC, JRK;
        double t=traj_goal.goal.trajectory.points[0].time_from_start.toSec();
        POS.resize(n_jts);  VEL.resize(n_jts);
        ACC.resize(n_jts);  JRK.resize(n_jts);
        while(t< sync_times.back() ){
            rbt_traj.sample_bezier_robot_trajectory( t, pos, vel, acc, jrk );
            for (int jt=0; jt<n_jts; jt++) {
                POS[jt].push_back( pos[jt]);
                VEL[jt].push_back(vel[jt]);
                ACC[jt].push_back( acc[jt]);
                JRK[jt].push_back( jrk[jt]);
            }
            sampled_time.push_back(t);
            t+= 1/frq;
        }

        // plot all joit trajectories
        //extract the original waypoint to check they are synchronized or not
//        std::vector< std::vector<double> > Pos_jt_wpt;
//        Pos_jt_wpt.resize(n_jts);
//        for(int jt=0; jt<n_jts; jt++)
//            for(int pt=0; pt<n_pts; pt++)
//                Pos_jt_wpt[jt].push_back( traj_goal.goal.trajectory.points[pt].positions[jt] );

        for (int jt=0; jt<n_jts; jt++) {
            plt::figure(19);
            plt::subplot(2, 2, 1);
            plt::named_plot( "pos",sampled_time, POS[jt]);
            plt::named_plot( "waypoints", sync_times, new_pos[jt], "r*");
            plt::title("pos"); plt::grid(true); plt::title("pos"); //plt::legend();
            plt::subplot(2, 2, 2);
            plt::named_plot( "vel",new_times[jt], new_vel[jt], "r*"); plt::grid(true); plt::title("vel");
            plt::named_plot( "vel",sampled_time, VEL[jt]); plt::grid(true); plt::title("vel");
            plt::subplot(2, 2, 3);
            plt::named_plot( "acc",new_times[jt], new_acc[jt], "r*"); plt::grid(true); plt::title("acc");
            plt::named_plot( "acc",sampled_time, ACC[jt]); plt::grid(true); plt::title("acc");
            plt::subplot(2, 2, 4);
            plt::named_plot( "jrk",sampled_time, JRK[jt]); plt::grid(true); plt::title("jrk");
        }

//        rbt_traj.print_attributes();
        plt::show();

    }
} //end of main






// tis function to remove 2nd and 2nd last way points from the trajectory
// this could be used or not, these two points were add by the spline parameterization part of moveit
trajectory_msgs::JointTrajectory remove_2nd_2ndlast_pts(trajectory_msgs::JointTrajectory trj_msg){
    int n_jts = trj_msg.joint_names.size();
    int org_n_pts = trj_msg.points.size();
    int n_pts = org_n_pts-2;
    std::cout<<"#original: jts, n_pts: "<< n_jts << ",  "<< org_n_pts << ", new_n_pts "<< n_pts <<std::endl;
    trajectory_msgs::JointTrajectory new_msg;
    new_msg.header = trj_msg.header;
    new_msg.joint_names = trj_msg.joint_names;
    new_msg.points.clear();

    for(int pt=0; pt<org_n_pts; pt++){
        if(pt == 1 || pt==org_n_pts-2)
            continue;
        std::cout<<"  "<< pt;
        new_msg.points.push_back( trj_msg.points[pt] ); //57.2958*
    }
     n_jts = new_msg.joint_names.size();
     n_pts = new_msg.points.size();
    std::cout<<"\n#updated: jts, n_pts: "<< n_jts << ",  "<< n_pts <<std::endl;
    return new_msg;
}








