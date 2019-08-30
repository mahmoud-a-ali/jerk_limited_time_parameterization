/**
\file   QuinticSplineTrajectory.h
\brief  definition of QuinticSplineTrajectory Class
 * this class is used to genrate limited jerk time parameteriztion for single joint
 * default controller/interpolation frequency: 125.
\author  Mahmoud Ali
\date    30/8/2019
*/



#include "quinticsplinetrajectory.h"


QuinticSplineTrajectory::QuinticSplineTrajectory()
{

}



void QuinticSplineTrajectory::init(std::vector<double> waypoints_pos, double init_vel, double init_acc, double final_vel, double final_acc){
//    std::cout<<"initiate QuinticSplineTrajectory ... "<<std::endl;
    if( waypoints_pos.empty() )
      throw(std::invalid_argument("QuinticSplineTrajectory can't be constructed: waypoints_pos are empty "));
    else{
        //initiate trajectory attributes
        positions_ = waypoints_pos;
        init_vel_=init_vel;  final_vel_=final_vel;
        init_acc_=init_acc;  final_acc_=final_acc;

        n_points_ = waypoints_pos.size();
        n_segs_ = n_points_ -1;
        n_eqs_ = 6*(n_points_ - 1);
        cof_mtrx_.resize(6);

        times_.resize(n_points_);
        durations_.resize(n_segs_);

        cal_positions_.resize(n_points_);
        cal_velocities_.resize(n_points_);
        cal_accelerations_.resize(n_points_);
        cal_jerks_.resize(n_points_);
        // cal_XX:  means calculated using the quintic coefficients result of matrix inversion
        VectorXd B(n_eqs_), X(n_eqs_);
        MatrixXd A =   MatrixXd::Zero(n_eqs_, n_eqs_);
        A_mtrx_=A; B_vec_=B; X_vec_=X;

    }
//    std::cout<<"QuinticSplineTrajectory has been initialized ... "<<std::endl;
}

// ===========================set absolute limits for pos, vel, acc, jrk====================
void QuinticSplineTrajectory::set_absolute_limits(double max_pos, double max_vel, double max_acc, double max_jrk){
    if(max_pos < 0 || max_vel < 0 || max_acc < 0|| max_jrk < 0)
        throw(std::invalid_argument("segment can't be constructed: absolute limits should be positive"));
    max_pos_ = max_pos;
    max_vel_ = max_vel;
    max_acc_ = max_acc;
    max_jrk_ = max_jrk;
}

//========================== calculate init duration time=pos/vel ================
void QuinticSplineTrajectory::set_init_duration(){
//    std::cout<<"\n_____________________ set_init_duration _____________________\n";
    int factor=0;
    VectorXd  duration(n_segs_);
      times_[0] = 0.0;
      for (int i = 0; i < n_segs_; ++i) {
          duration(i) = fabs( (fabs(positions_[i+1]) - fabs(positions_[i])) ) /max_vel_;
          factor = duration(i)/0.008;
          duration(i)= (factor+1) * 0.008;
          durations_[i] = duration(i); // update_duration
          times_[i+1] = times_[i] + durations_[i]; // update_times
      }
      dur_vec_ = duration;  // update dur_vector
      // here probably we can set dur for first and last seg in seperate way if it is start from zero state(vel, acc)
}


//=======================set duration to any values (external), mostly the sync duration from robot_traj class ============
void  QuinticSplineTrajectory::set_durations( std::vector<double> durations){
    durations_ = durations;
    if(times_.size() != (durations_.size()+1))
        std::cout<<"warn: size_times != size_dur+1"<< std::endl;
    times_[0] = t_start_;
    for (int i = 0; i < n_points_-1; ++i) {
        times_[i+1] = times_[i] + durations_[i];
        dur_vec_(i) = durations[i]; // this the one using for matrx and vector calculation
    }
}



//===================== formula to generate C1 matrix ============================
// generate C1 matrix which is depends on the time of each segment,
//idx is segment indx, h vectore contains segments times
void QuinticSplineTrajectory::generate_coef_matrix_c1(const int idx, const VectorXd &dur, MatrixXd &C1){
//    std::cout<<"\n_____________________ generate_coef_matrix_c1 _____________________\n";
    MatrixXd C(5,6);
    C<< 1,  dur(idx),  pow(dur(idx), 2),    pow(dur(idx), 3),    pow(dur(idx), 4),     pow(dur(idx), 5),
        0,         1,        2*dur(idx),  3*pow(dur(idx), 2),  4*pow(dur(idx), 3),   5*pow(dur(idx), 4),
        0,         0,                 1,          3*dur(idx),  6*pow(dur(idx), 2),  10*pow(dur(idx), 3),
        0,         0,                 0,                   1,          4*dur(idx),  10*pow(dur(idx), 2),
        0,         0,                 0,                   0,                   1,           5*dur(idx);
    C1 = C;
}



//========================== solve for vel, acc, times given the pos ====================
void  QuinticSplineTrajectory::solve_for_vel_acc_times( ){
//    std::cout<<"\n_____________________ solve_for_vel_acc_times() _____________________\n";
    if( max_pos_>0 && max_vel_>0 && max_acc_>0 && max_jrk_>0)
        solve_for_vel_acc_times(max_pos_, max_vel_, max_acc_, max_jrk_);
    else
        throw( std::invalid_argument("absolute limits (pos, vel, acc, jrk) should be positive and non-zero values") );
}
//============================ another footprint with setting new limits ====================
void QuinticSplineTrajectory::solve_for_vel_acc_times( double max_pos, double max_vel, double max_acc, double max_jrk ){
//    std::cout<<"\n_____________________ solve_for_vel_acc_times(...) _____________________\n";
    set_absolute_limits( max_pos, max_vel, max_acc, max_jrk);
    set_init_duration();
    solve_for_quintic_coef( ); //uses current values of attributes times_duration_dur_vec to solve for X_vec and w_pts pos, vel, acc, jrk
}
//========================== given sync_times(required), solve for vel, acc given the pos ====================
void QuinticSplineTrajectory::solve_for_vel_acc( std::vector<double> sync_dur ){
//    std::cout<<"\n_____________________ solve_for_vel_acc(...) _____________________\n";
    set_durations(sync_dur);
    solve_for_quintic_coef( );
}




//=================given durations&pos, fill matrix A and vector B then solve for X, as AX=B ====================
void QuinticSplineTrajectory::solve_for_quintic_coef( ){ // it uses current values of dur_vec, and times_
//    std::cout<<"\n_____________________ solve_for_quintic_coef _____________________\n";

    // next step to fill matrices A, and B:
    //given duration solve for vel acc
    MatrixXd C1 =  MatrixXd::Zero(5, 6);
    MatrixXd C2 =  MatrixXd::Zero(5, 6);
    for (int i=0; i<5; i++)
        C2(i,i)=-1.0;

    // >>>>>>>>>>>step_1:  3 boundary conditions for first point (pos0, vel0, acc0), 3 rows
    for (int i=0; i<3; i++)
        A_mtrx_(i,i)= 1.0;

    //>>>>>>>>>>> step_2:  pos condition for all waypoints except first and last, (n-2) rows
     int col_idx= 6;
     for (int r = 3; r < 3+(n_points_-2); ++r) {  //start filling from 4th row, #rows= n-2
              A_mtrx_(r,col_idx) = 1;
              col_idx+=6;
      }  // n-2 rows are filled so far, so total rows are n-2+3= n+1


     //>>>>>>>>>>>step_3: boundary conditions for last point (posn, veln, accn), 3 rows (depends on time so it is inside the loop)
         for (int r = n_points_+1 ; r < 3+(n_points_+1); ++r) {// for 3 row
             for (int c = n_eqs_-6 ; c < n_eqs_; ++c) {  // last 6 columns
                generate_coef_matrix_c1( n_points_-2, dur_vec_, C1);
                A_mtrx_(r, c) = C1(r-(n_points_+1), c-(n_eqs_-6));     // last h ==> hn-2
             }
         }//3+(n-2)+3 = n+4 rows are filled so far,  so next row to fill is (n+5)th row [its is index= n+4]


     //>>>>>>>>>>>step_4: 5(n-2) coninuity conditions for each waypoints except first and last, 5(n-2) rows (depend on time so it is inside the loop)
     // for each intermdiate waypoint (all except first and last waypoints), #blks= n-2
        int row_idx = n_points_+4; col_idx=0;  //start filling from (n+2)th row [its is index= n+1],
        for (int blk=0; blk< n_points_-2; blk++) { // n-2 blks, each blk is 5 equations with 6 coef
            for (int r = 0 ; r < 5 ; ++r) {  // 5(n-2) rows
                for (int c = 0 ; c < 6; ++c) {  // last 6 columns
     //               ROS_INFO_STREAM("r: "<< r <<"   c:"<< c);
                     generate_coef_matrix_c1( blk, dur_vec_, C1);
                     A_mtrx_(row_idx+r, col_idx+c)   = C1(r,c);     // last h ==> hn-2
                     A_mtrx_(row_idx+r, col_idx+c+6) = C2(r,c);
                }
             }
            col_idx += 6;
            row_idx += 5;
         } //all row s are filled

    //>>>>>>>>>>>step_5: fill B vector B
       // initial condtions pos0 vel0 acc0
       B_vec_(0) = positions_[0];
       B_vec_(1) = init_vel_;
       B_vec_(2) = init_acc_; //  3 rows so far
       //intermediate pos pos1, pos2, ..... till pos(n-1)
       for (int i=1; i< n_points_-1; i++) // all pos from pos1 to pos(n-1)= (n-2) rows
           B_vec_(i+2)= positions_[i]; // so far 3+n-2 = n+1
       // final condtions posn veln accn
       B_vec_(n_points_+1) = positions_[n_points_-1];
       B_vec_(n_points_+2) = final_vel_;
       B_vec_(n_points_+3) = final_acc_;
      // rest are zeros it is from equating equations
       for (int i=n_points_+4; i< n_eqs_; i++) // last 5(n-2) rows
           B_vec_(i)= 0; // so far 3+n-2 = n+1

     //>>>>>>>>>>>step_6: find solution for X using matirx inverse
        HouseholderQR<MatrixXd> qr(A_mtrx_);
        X_vec_ = qr.solve(B_vec_); // computes A^-1 * b
//        std::cout<<"sol: \n"<<X_vec_<<std::endl;

        std::vector<double> a, b, c, d, e, f;
        for (int i = 0; i < n_eqs_; i+=6) {
            a.push_back( X_vec_(i) );
            b.push_back( X_vec_(i+1) );
            c.push_back( X_vec_(i+2) );
            d.push_back( X_vec_(i+3) );
            e.push_back( X_vec_(i+4) );
            f.push_back( X_vec_(i+5) );
       }
        cof_mtrx_[0] = a;
        cof_mtrx_[1] = b;
        cof_mtrx_[2] = c;
        cof_mtrx_[3] = d;
        cof_mtrx_[4] = e;
        cof_mtrx_[5] = f;

        std::vector<double> state;
        for (int pt = 0; pt < n_points_; ++pt) {
           sample(times_, times_[pt], cof_mtrx_, state);
           cal_positions_[pt]      =  state[0] ;
           cal_velocities_[pt]    =  state[1];
           cal_accelerations_[pt] =  state[2] ;
           cal_jerks_[pt]         =  state[3] ;
//           std::cout<< "\n pt_"<<pt<<": p="<< state[0]<<", v="<< state[1]<<", a="<< state[2]<<", j="<< state[3] ;//<<std::endl ;
        }
}



//=============find segment number for each corresponding time, required for sampling ================
// finds in which segment time instant tg belons to
int QuinticSplineTrajectory::find_seg_number (const double &tg, const std::vector<double> &T_vec){
//    std::cout<<"\n_____________________ find_seg_number _____________________\n";
    // segment changes from 0 to n-2 which mean n-1 seg
    int seg = 0;
    if(tg<= T_vec[0]) //less than tstart
        return seg;
    else if(tg>= T_vec[T_vec.size() -2 ] ) //if tg is greater than the starting time of the last segment
        return T_vec.size() - 2;
    else {
        for (int i=0; i< T_vec.size()-2 ; i++) { // between tstart and tend
            if(tg>= T_vec[i] && tg< T_vec[i+1])
                seg = i;
        }
        return seg;
    }
}



//=============== sample trajectory, find state(pos, vel, acc, jrk) at a given time tg ==================
//T_vec is time vector of waypoints, cof is coef matrix of quintic spline
int QuinticSplineTrajectory::sample (const std::vector<double>& T_vec, const double& tg, const std::vector<std::vector<double>> &cof, std::vector<double> & state){//, bool pos_plt=true, bool vel_plt=true, bool acc_plt=true, bool jrk_plt=true ){
//    std::cout<<"\n_____________________ sample _____________________\n";
    state.resize(4);
    std::vector<double> a=cof[0], b=cof[1], c=cof[2], d=cof[3], e=cof[4], f=cof[5];
    double pos=0, vel=0, acc=0, jrk=0;
    int n = T_vec.size(), seg=0; // n:number of points
    double t=0; // to represent tg-tseg
    //check for vector sizes
//    std::cout<<"\nsize of a= "<< a.size() << " size of T= "<< T_vec.size();
    if( a.size() != T_vec.size()-1 )
        std::cout<<"error: coef and time have different size\n";

    // check for tg inside T_vec or not, if yes find corrresponding segment
    if(tg < T_vec[0]){
        std::cout<<"Warn: request time instant is before start time of trajectory\n";
        pos= a[0];
        vel= b[0];
        acc= 2*c[0];
        jrk= 6*d[0];
    }
    else if(tg > T_vec[n-1] ){
        std::cout<<"Warn: request time instant is after the end time of trajectory\n ";
        t = tg - T_vec[n-1];
        pos=   a[n-2] +   b[n-2]*t +         c[n-2]*pow( t, 2) +        d[n-2]*pow( t, 3) +    e[n-2]*pow( t, 4) +    f[n-2]* pow( t, 5)  ;
        vel=              b[n-2]        +  2*c[n-2]*t          +      3*d[n-2]*pow( t, 2) +  4*e[n-2]*pow( t, 3) +  5*f[n-2]* pow( t, 4)  ;
        acc=                               2*c[n-2]                 + 6*d[n-2]*t          + 12*e[n-2]*pow( t, 2) + 20*f[n-2]* pow( t, 3)  ;
        jrk=                                                          6*d[n-2]                 + 24*e[n-2]*t          + 60*f[n-2]* pow( t, 2)  ;
    }
    else{
        seg= find_seg_number(tg, T_vec);
//        std::cout<<"###seg: "<<seg<< std::endl;
        t = tg - T_vec[seg];
//        std::cout<<"      tg= " <<tg<< "   seg_number= "<<  seg<<"   t= "<<t<<std::endl;
        pos=   a[seg] +   b[seg]*t +    c[seg]*pow( t, 2) +   d[seg]*pow( t, 3) +    e[seg]*pow( t, 4) +    f[seg]* pow( t, 5) ;
        vel=              b[seg]        +  2*c[seg]*t          + 3*d[seg]*pow( t, 2) +  4*e[seg]*pow( t, 3) +  5*f[seg]* pow( t, 4) ;
        acc=                               2*c[seg]                 + 6*d[seg]*t          + 12*e[seg]*pow( t, 2) + 20*f[seg]* pow( t, 3) ;
        jrk=                                                          6*d[seg]                 + 24*e[seg]*t          + 60*f[seg]* pow( t, 2) ;
    }
    // assign state to state vector
    state[0] = pos;
    state[1] = vel;
    state[2] = acc;
    state[3] = jrk;
    return seg;
}



//======== check non_lmtd or non_mono sgmnts =============================
// this returns back segment that are not limited or segemnts that are not montonic
bool QuinticSplineTrajectory::check_behavoir(std::vector<int> &unlimited_seg_id, std::vector<int> &nonmono_seg_id){
    unlimited_seg_id.clear();
    nonmono_seg_id.clear();
    std::vector<double> start_state, end_state;
    start_state.resize(3);
    end_state.resize(3);
    bool mono=false,lmtd_mono=true;
    bool vel_lmt=false, acc_lmt=false, jrk_lmt=false;

    // for each segment, instanitiate an object of the sgmnt class
    for (int seg=0; seg< n_segs_; seg++) {
        start_state[0] = cal_positions_[seg];
        start_state[1] = cal_velocities_[seg];
        start_state[2] = cal_accelerations_[seg];

        end_state[0] = cal_positions_[seg+1];
        end_state[1] = cal_velocities_[seg+1];
        end_state[2] = cal_accelerations_[seg+1];

        BezierQuinticSegment sg( times_[seg],  durations_[seg], start_state, end_state);
        sg.set_absolute_limits( max_pos_, max_vel_, max_acc_, max_jrk_ );
        sg.compute_coef();
        sg.compute_maxmin_times();
        mono= sg.check_monotonic();
        jrk_lmt= sg.check_jerk_limit();
        acc_lmt= sg.check_acc_limit();
        vel_lmt= sg.check_vel_limit();
        if( !vel_lmt || !acc_lmt || !jrk_lmt ){
//              std::cout<<"seg_" <<seg <<": not_limited behavoir ! " <<std::endl;
                unlimited_seg_id.push_back( seg);
                lmtd_mono= false;
         }

        if(!mono){
//          std::cout<<"seg_" <<seg <<": non_mono behavoir ! " <<std::endl;
            nonmono_seg_id.push_back( seg);
            lmtd_mono=false;
        }
    }
//    std::cout<<"lmtd_mono: " <<lmtd_mono<<std::endl;
    return lmtd_mono;
}




//========================= update waypoint times till reach lmtd behavoir ================
std::vector<double> QuinticSplineTrajectory::update_waypoints_times(){
    //    std::cout<<"=======> update_waypoints_times: .... "<<std::endl;

    int count=0; // just to check max number of iteration
    bool update_all_seg_time= false; // one option could be valid
    bool monotonic_pos=false, lmtd_jrk = false, lmted_mono=false; //will be set to true when all jrk values are in the limits
    std::vector<int> unlimited_seg_idx, non_mono_seg_idx;
    std::vector<std::vector<double>> cof ;
    solve_for_vel_acc_times();
    std::vector<double> dur = durations_;
    while (!lmtd_jrk ){//!lmtd_jrk || !monotonic_pos ){
//        if(count>=100)
//            break; // if you want to set max number of iteration
        set_durations(dur);
        solve_for_quintic_coef();
//        std::cout<<"\n_____________ check behavoir main ____________: "<<std::endl;
        lmted_mono=  check_behavoir( unlimited_seg_idx, non_mono_seg_idx);
        // here we have both segment that are not limited and not monotonic
        // currently only limited jerk condition is being used
         if(!lmted_mono){
             if(!unlimited_seg_idx.empty()){
//                 std::cout<<"\n_____________unlimited_seg_idx____________: "<<std::endl;
                 for (int i=0; i< unlimited_seg_idx.size(); i++){
//                     std::cout<<"  "<< unlimited_seg_idx[i];
                     dur[ unlimited_seg_idx[i] ] += 0.008;
                 }
//                 std::cout<<".";
             }else {
//                 std::cout<<"\n_____________it is limited !_____________"<<std::endl;
                 lmtd_jrk = true;  monotonic_pos =true;
             }
             unlimited_seg_idx.clear();
             //--------------------------------------------------------------------
             // this part for checking monotonic behavoir
//            if(!non_mono_seg_idx.empty()){
//                std::cout<<"\n_____________ non_mono_seg_idx ____________: "<<std::endl;
//                for (int i=0; i< non_mono_seg_idx.size(); i++){
//                    std::cout<<"   "<< non_mono_seg_idx[i];
////                    dur[ non_mono_seg_idx[i] ] += 0.008;
////                    if(dur[ non_mono_seg_idx[i] ] >0)
////                        dur[ non_mono_seg_idx[i]-1 ] += 0.008;
////                    if(dur[ non_mono_seg_idx[i] ] < dur.size()-1 )
////                        dur[ non_mono_seg_idx[i]+1 ] += 0.008;
////                      for (int f=0; f< dur.size(); f++)
////                        dur[ f ] += 0.008;
//                }
//                std::cout<<std::endl;
//            }else {
//                std::cout<<"\n_____________it is mono!_____________"<<std::endl;
//                monotonic_pos =true;
//            }
//            non_mono_seg_idx.clear();
         }
//         else
//             std::cout<<"\n_____________it is mono&limited !_____________"<<std::endl;

    count++;
    //    std::cout<<"#loops= "<<count;
//        std::cout<<"  "<<count;

        if(lmtd_jrk){ // lmted_mono: chck for both lmtd and mono,  lmtd_jrk: checks only for limited condition
            return times_;
        }
    }
}



//=====================================print attributes =======================
void QuinticSplineTrajectory::print_attributes(){
    std::cout<<"\n_____________________ print_attributes _____________________\n";

    std::cout<<"###limits: \n" <<"pos: " <<max_pos_ <<",  vel: " <<max_vel_<<", acc: " <<max_acc_<<",  jrk: " <<max_jrk_<<std::endl;

    std::cout<< "###positions_:  "<<std::endl;
    for (int i=0; i<positions_.size(); i++)
        std::cout<< positions_[i]<<std::endl;


    std::cout<< "###cal_positions_:  "<<std::endl;
    for (int i=0; i<cal_positions_.size(); i++)
        std::cout<< cal_positions_[i]<<std::endl;

    std::cout<< "###cal_velocities_:  "<<std::endl;
    for (int i=0; i<cal_velocities_.size(); i++)
        std::cout<< cal_velocities_[i]<<std::endl;

    std::cout<< "###cal_accelerations_:  "<<std::endl;
    for (int i=0; i<cal_accelerations_.size(); i++)
        std::cout<< cal_accelerations_[i]<<std::endl;

    std::cout<< "###cal_jerks_:  "<<std::endl;
    for (int i=0; i<cal_jerks_.size(); i++)
        std::cout<< cal_jerks_[i]<<std::endl;

    std::cout<< "###cal_durations_:  "<<std::endl;
    for (int i=0; i<durations_.size(); i++)
        std::cout<< durations_[i]<<std::endl;

    std::cout<< "###times_:  "<<std::endl;
    for (int i=0; i<times_.size(); i++)
        std::cout<< times_[i]<<std::endl;

    std::cout<< "###a:  "<<std::endl;
    for (int i=0; i<cof_mtrx_[0].size(); i++)
        std::cout<< cof_mtrx_[0][i]<<std::endl;


//    std::cout<<    "###A_matrix: \n"<<A_ <<std::endl;
    std::cout<<   "###B_vector: \n"<<B_vec_ <<std::endl;

}






