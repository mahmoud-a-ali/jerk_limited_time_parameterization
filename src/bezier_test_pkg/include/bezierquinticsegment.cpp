/**
\file   BezierQuinticSegment.h
\brief  definition of BezierQuinticSegment Class
 * this class uses bezier curve intepolation to calculate duration for a segment
 * from generic state to generic state, while keeping vel, acc, and jerk in limits
 * default frequency: 125.
\author  Mahmoud Ali
\date    30/8/2019
*/



#include "bezierquinticsegment.h"

BezierQuinticSegment::BezierQuinticSegment()
{

}

//=========================init segment with required values ============================
void BezierQuinticSegment::init(double start_time, double duration, std::vector<double> start_state, std::vector<double> end_state){
//    std::cout<<">>> initialize segment ... "<<std::endl;

    if (start_time < 0)
      throw(std::invalid_argument("segment can't be constructed: start_time should be positive "));

    if ( duration < 0)
      throw(std::invalid_argument("segment can't be constructed: duration should be positive."));

    if (start_state.empty() || end_state.empty())
      throw(std::invalid_argument("segment can't be constructed: start & end points  can't be empty."));

    if (start_state.size() != 3)
      throw(std::invalid_argument("segment can't be constructed: initial state/conditions should have 3 values (pos, vel, acc)"));

    if (end_state.size() != 3)
      throw(std::invalid_argument("segment can't be constructed: final state/conditions should have 3 values (pos, vel, acc)"));

    t_start_ = start_time;
    set_duration_( duration);
    start_state_ = start_state;
    end_state_ = end_state;
    coef_.resize(6);
    iter_=0;
//    std::cout<<"segment has been initialized ... "<<std::endl;
}


//===================================set limits================================
void BezierQuinticSegment::set_absolute_limits(double max_pos, double max_vel, double max_acc, double max_jrk){
    if(max_pos < 0 || max_vel < 0 || max_acc < 0|| max_jrk < 0)
        throw(std::invalid_argument("segment can't be constructed: absolute limits should be positive"));
    max_pos_ = max_pos;
    max_vel_ = max_vel;
    max_acc_ = max_acc;
    max_jrk_ = max_jrk;
}



//=============================== update duration ================================
double  BezierQuinticSegment::update_duration( ){
    if( max_pos_>0 && max_vel_>0 && max_acc_>0 && max_jrk_>0)
        return update_duration(max_pos_, max_vel_, max_acc_, max_jrk_);
    else
        throw( std::invalid_argument("absolute limits (pos, vel, acc, jrk) should be positive and non-zero values") );
}
double  BezierQuinticSegment::update_duration(double max_pos, double max_vel, double max_acc, double max_jrk){
//    std::cout<<"\n---------------------update duration-------------------"<< std::endl;
    set_absolute_limits( max_pos,  max_vel,  max_acc,  max_jrk);
    compute_coef();
    compute_maxmin_times();

    bool mono = check_monotonic();
    bool lmtd = (check_jerk_limit() && check_acc_limit() && check_vel_limit() );
//    std::cout<<"lmted: "<< lmtd <<",  mono : "<< mono << std::endl;
    //------------------------------------ case 1 lmtd & mono ------------------------------
    if( (check_jerk_limit() && check_acc_limit() && check_vel_limit() )  && check_monotonic()){
//        std::cout<<" requisted time already grantee jrk, pos, vel limits, and grantee montonic behavoir ! " <<std::endl;
        return duration_;
    }
    else{ // then it is one of three cases

//        while( !( (check_jerk_limit() && check_acc_limit() && check_vel_limit() )  && check_monotonic())  ){
//             std::cout<<">>>>>>>>>>>>> retiming........."<< std::endl;
            //------------------------------------ case 2 not_lmtd but mono ------------------------------
             if( !(check_jerk_limit() && check_acc_limit() && check_vel_limit() ) && check_monotonic()){
//                std::cout<<" WARN:: these times and states gives not_limited / monotonic behavoir " <<std::endl;
                while ( !(check_jerk_limit() && check_acc_limit() && check_vel_limit() )  ){
                    set_duration_( duration_ + 0.01);
                    compute_coef();
                    compute_maxmin_times();
                    if( ! check_monotonic() )
                        break;
                    iter_ ++;
                }
            }
            //------------------------------------ case 3 not_lmtd & not_mono ------------------------------
            else if( !(check_jerk_limit() && check_acc_limit() && check_vel_limit() ) && !check_monotonic()){
//                 std::cout<<" WARN:: these times and states gives not_limited / not_monotonic behavoir  "<<std::endl;
                 while ( !check_monotonic()  ){
                     end_state_[1] = end_state_[1] - ( end_state_[1] >0 ? 1 : -1 );
                     compute_coef();
                     compute_maxmin_times();
                     if( check_monotonic() )
                         break;
                     iter_ ++;
                 }
             }
            //------------------------------------ case 4 lmtd but not_mono ------------------------------
             else if( (check_jerk_limit() && check_acc_limit() && check_vel_limit() ) && !check_monotonic()){
//                  std::cout<<" WARN:: these times and states gives limited / not_monotonic behavoir  ! " <<std::endl;
                  int dir = start_state_[1]>0 ? 1: -1 , new_dir = dir;
                  while ( ( new_dir == dir ) && !check_monotonic()  ){
                      end_state_[1] = end_state_[1] - ( end_state_[1] >0 ? 1 : -1 );
//                      std::cout<<"              new_end_vel:  "<< end_state_[1] <<std::endl;

                      new_dir = end_state_[1]>0 ? 1: -1 ;
                      compute_coef();
                      compute_maxmin_times();
                      if( ! (check_jerk_limit() && check_acc_limit() && check_vel_limit() ) )
                          break;
                      iter_ ++;
                  }
                  while ( new_dir != dir ){
                       end_state_[1] = 0;
                       compute_coef();
                       compute_maxmin_times();
                  }

              }
        } //end the while condition
//    } //end of else the 3 later cases
    return  duration_;
}



//=============================== compute coef ===============================
void BezierQuinticSegment::compute_coef(){
    coef_.resize(6);
    double t1=t_start_, t2=t_end_;
    double p0=start_state_[0], v0=start_state_[1], a0=start_state_[2], pn=end_state_[0], vn=end_state_[1], an=end_state_[2];
    coef_[0] = -p0/pow((t1 - t2), 5) ;
    coef_[1] = -(5*p0 - t1*v0 + t2*v0)/(5*pow((t1 - t2),5));
    coef_[2] = -(a0*t1*t1 - 2*a0*t1*t2 - 8*v0*t1 + a0*t2*t2 + 8*v0*t2 + 20*p0)/(20*pow((t1 - t2), 5));
    coef_[3] = -(an*t1*t1 - 2*an*t1*t2 + 8*vn*t1 + an*t2*t2 - 8*vn*t2 + 20*pn)/(20*pow((t1 - t2), 5));
    coef_[4] = -(5*pn + t1*vn - t2*vn)/(5*pow((t1 - t2), 5));
    coef_[5] = -pn/pow((t1 - t2), 5);
}


//=======================check vel, acc, jrk limits/voilation ========================
bool BezierQuinticSegment::check_jerk_limit(){
    double P0=coef_[0], P1=coef_[1], P2=coef_[2], P3=coef_[3], P4=coef_[4], P5=coef_[5];
    double t1=t_start_, t2=t_end_;
    double tm[]= {t_start_, t_end_, tj_, ta1_, ta2_, tv1_, tv2_, tv3_};
    double jrk =0;
     for(auto t : tm){
        jrk = 180*P1*pow( (t-t2), 2) - 60*P0*pow( (t-t2), 2) - 60*P2*pow( (t-t1), 2) - 180*P2*pow( (t-t2), 2) + 180*P3*pow( (t-t1), 2) + 60*P3*pow( (t-t2), 2) - 180*P4*pow( (t-t1), 2) + 60*P5*pow( (t-t1), 2) + 60*P1*(2*t - 2*t2)*(t - t1) - 90*P2*(2*t - 2*t1)*(2*t - 2*t2) + 90*P3*(2*t - 2*t1)*(2*t - 2*t2) - 12*P4*(2*t - 2*t1)*(5*t - 5*t2);
//        std::cout<<"check_jrk: "<< t <<"   check_jrk: "<< jrk<<std::endl;
        if(fabs(jrk) > max_jrk_)
           return false;
     }
     return true;
}

bool BezierQuinticSegment::check_acc_limit(){
    double P0=coef_[0], P1=coef_[1], P2=coef_[2], P3=coef_[3], P4=coef_[4], P5=coef_[5];
    double t1=t_start_, t2=t_end_;
    double tm[]= {t_start_, t_end_, tj_, ta1_, ta2_, tv1_, tv2_, tv3_};
    double acc =0;
     for(auto t : tm){
         acc = 40*P1*pow( (t-t2), 3) - 20*P0*pow( (t-t2), 3) - 20*P2*pow( (t-t2), 3) + 20*P3*pow( (t-t1), 3) - 40*P4*pow( (t-t1), 3) + 20*P5*pow( (t-t1), 3) + 60*P1*(t - t1)*pow( (t-t2), 2) - 60*P2*(2*t - 2*t1)*pow( (t-t2), 2) - 30*P2*(2*t - 2*t2)*pow( (t-t1), 2) + 30*P3*(2*t - 2*t1)*pow( (t-t2), 2) + 60*P3*(2*t - 2*t2)*pow( (t-t1), 2) - 12*P4*(5*t - 5*t2)*pow( (t-t1), 2);
//         std::cout<<"check_acc: "<< t<<"     check_acc: "<< acc<<std::endl;
         if(fabs(acc) > max_acc_)
           return false;
     }
     return true;
}


bool BezierQuinticSegment::check_vel_limit(){
    double P0=coef_[0], P1=coef_[1], P2=coef_[2], P3=coef_[3], P4=coef_[4], P5=coef_[5];
    double t1=t_start_, t2=t_end_;
    double tm[]= {t_start_, t_end_, tj_, ta1_, ta2_, tv1_, tv2_, tv3_};
    double vel =0;
    for(auto t: tm){
         vel = 5*P1*pow( (t-t2), 4) - 5*P0*pow( (t-t2), 4) - 5*P4*pow( (t-t1), 4) + 5*P5*pow( (t-t1), 4) + 20*P1*(t - t1)*pow( (t-t2), 3) - 10*P2*(2*t - 2*t1)*pow( (t-t2), 3) + 10*P3*(2*t - 2*t2)*pow( (t-t1), 3) - 4*P4*(5*t - 5*t2)*pow( (t-t1), 3) - 30*P2*pow( (t-t1), 2)*pow( (t-t2), 2) + 30*P3*pow( (t-t1), 2)*pow( (t-t2), 2);
    //    std::cout<<"check_vel: "<< t <<"     check_vel: "<< vel<<std::endl;
        if(fabs(vel) > max_vel_)
            return false;
     }
     return true;
}





//=========================== check zero transition ============================
bool BezierQuinticSegment::check_zero_transition(){
    if( fabs(start_state_[1])<0.001 && fabs(start_state_[2])<0.001 && fabs(end_state_[1])<0.001 && fabs(end_state_[2])<0.00001 )
        return true;
    else
        return false;
}


//====================compute min duration ========================================
// calculate min  duration
double BezierQuinticSegment::compute_min_duration( ){
    if( max_pos_>0 && max_vel_>0 && max_acc_>0 && max_jrk_>0)
        return compute_min_duration(max_pos_, max_vel_, max_acc_, max_jrk_);
    else
        throw( std::invalid_argument("absolute limits (pos, vel, acc, jrk) should be positive and non-zero values") );
}
double BezierQuinticSegment::compute_min_duration(double max_pos, double max_vel, double max_acc, double max_jrk){
    set_duration_(  0.01);
     //starting with one time sample
    return update_duration();
}



//===================== calculate max  duration =================================
double BezierQuinticSegment::compute_max_duration(){
    if( max_pos_>0 && max_vel_>0 && max_acc_>0 && max_jrk_>0)
        return compute_max_duration(max_pos_, max_vel_, max_acc_, max_jrk_);
    else
        throw( std::invalid_argument("absolute limits (pos, vel, acc, jrk) should be positive and non-zero values") );
}

double BezierQuinticSegment::compute_max_duration(double max_pos, double max_vel, double max_acc, double max_jrk){
    if( check_zero_transition()){
//        std::cout<<"segment transition is from zero state(vel, acc) to zero state(vel , acc), so it has no max duration (consider 10e5)"<<std::endl;
        return 100000;
    }
    //    std::cout<<"calculate max duration*/..."<<std::endl;
    set_absolute_limits(max_pos, max_vel, max_acc, max_jrk);
    duration_ = compute_min_duration();
    compute_coef();
    compute_maxmin_times();
//    std::cout<<"==> calculate max duration, starting with min duration..."<< duration_<<std::endl;
//    std::cout<<"check_monotonic() ..."<< check_monotonic() <<std::endl;
    while(check_monotonic()){
        set_duration_( duration_ + 0.01);
        compute_coef();
        compute_maxmin_times();
    }
    // you choose to set duration to its max or not
//    duration_ = t_end_ - t_start_;
//    return t_end_ - t_start_;
    set_duration_( duration_ - 0.01);
    return duration_ ;
}



//======================================== check monotonic behavoir =================
bool BezierQuinticSegment::check_monotonic(){
//    compute_coef();
//    compute_maxmin_times();
    double eps = 0.000001;
    double P0=coef_[0], P1=coef_[1], P2=coef_[2], P3=coef_[3], P4=coef_[4], P5=coef_[5];
    double t1=t_start_, t2=t_end_;
    double tm[]= {tj_, ta1_, ta2_, tv1_, tv2_, tv3_};
    double reached_vel =0;
    bool mono = false;
    for(auto t: tm){
        reached_vel = 5*P1*pow( (t-t2), 4) - 5*P0*pow( (t-t2), 4) - 5*P4*pow( (t-t1), 4) + 5*P5*pow( (t-t1), 4) + 20*P1*(t - t1)*pow( (t-t2), 3) - 10*P2*(2*t - 2*t1)*pow( (t-t2), 3) + 10*P3*(2*t - 2*t2)*pow( (t-t1), 3) - 4*P4*(5*t - 5*t2)*pow( (t-t1), 3) - 30*P2*pow( (t-t1), 2)*pow( (t-t2), 2) + 30*P3*pow( (t-t1), 2)*pow( (t-t2), 2);
        if( (reached_vel >= -eps && start_state_[1]>=-eps  &&  end_state_[1]>= -eps) || (reached_vel <=eps && start_state_[1]<=eps  &&  end_state_[1]<=eps)  )
            mono= true;
        else{
//            std::cout<<"start_state_[1]: "<<start_state_[1]<<std::endl;
//            std::cout<<"end_state_[1]: "<<end_state_[1]<<std::endl;
//            std::cout<<"reached_vel: "<<reached_vel<<std::endl;
            return false;
        }
    }
    return mono;
}


//============================ find real roots for quadrature eq =============================
// to find real roots for quadrature eq (if it is imaginary or if a=0 and b=0 it returns 0,0)
int BezierQuinticSegment::quad_eq_real_root (const double &a, const double &b, const double &c, double &r1, double &r2)
{
    double disc= b*b - 4*a*c;
    if(a==0.000000){
        if(b==0.000000){
            r1= 0;
            r2= 0;
            return 0;
        }else {
            r1= -c/b;
            r2= -c/b;
            return 1;
        }
    }else {
        if(disc > 0){
            r1 = (-b -sqrt(disc) ) / (2*a);
            r2 = (-b +sqrt(disc) ) / (2*a);
            return 2;
        }else {
            r1= 0;
            r2= 0;
            return 0;
        }
    }
}

//======================== find the real root/roots of a cubic equation==============================
int  BezierQuinticSegment::cubic_eq_real_root (double a, double b, double c, double d, std::vector<double> &roots)
{
        roots.resize(3);
        if (a == 0.0000000)
        {
            throw(std::invalid_argument("The coefficient of the cube of x is 0. Please use the utility for a SECOND degree quadratic. No further action taken."));
            return 0;
        } //End if a == 0

        if (d == 0.0000000)
        {
//            throw(std::invalid_argument("One root is 0. Now divide through by x and use the utility for a SECOND degree quadratic to solve the resulting equation for the other two roots. No further action taken."));
//            return 0;
            roots[0]=0;
            double n_rts= quad_eq_real_root (a, b, c, roots[1],roots[2]);
            return n_rts+1;
        } //End if d == 0
//        std::cout<<"Eq info: "<<std::endl;
//        std::cout<< a <<", "<< b <<", "<< c <<", "<< d <<std::endl;
        b /= a;
        c /= a;
        d /= a;
        double disc, q, r, dum1, s, t, term1, r13;
        q = (3.0*c - (b*b))/9.0;
        r = -(27.0*d) + b*(9.0*c - 2.0*(b*b));
        r /= 54.0;
        disc = q*q*q + r*r;
        double root1 = 0; //The first root is always real.
        term1 = (b/3.0);
        if (disc > 1e-10) { // one root real, two are complex
//            std::cout<<"disc > 0, disc= "<< disc <<std::endl;
            s = r + sqrt(disc);
            s = ((s < 0) ? -pow(-s, (1.0/3.0)) : pow(s, (1.0/3.0)));
            t = r - sqrt(disc);
            t = ((t < 0) ? -pow(-t, (1.0/3.0)) : pow(t, (1.0/3.0)));
            double x1r= -term1 + s + t;
            term1 += (s + t)/2.0;
            double x3r = -term1,  x2r = -term1;
            term1 = sqrt(3.0)*(-t + s)/2;
            x2r = term1;
            double x3i = -term1;
            roots[0]= x1r;
            roots[1] = -100.0;
            roots[2]= -100.0;
            return 1;
        }
        // End if (disc > 0)
        // The remaining options are all real
        double x3i =0, x2r = 0;
        if (disc>=0.000 && disc< 1e-10){ // All roots real, at least two are equal.
//            std::cout<<"disc = 0 , disc= "<< disc <<std::endl;
            disc=0;
            r13 = ((r < 0) ? -pow(-r,(1.0/3.0)) : pow(r,(1.0/3.0)));
            double x1r= -term1 + 2.0*r13;
            double x3r = -(r13 + term1);
            x2r = -(r13 + term1);
            roots[0]= x1r;
            roots[1] = x2r;
            roots[2]= x3r;
            return 2;
        } // End if (disc == 0)
        // Only option left is that all roots are real and unequal (to get here, q < 0)
//        std::cout<<"disc < 0 , disc= "<< disc <<std::endl;
        q = -q;
        dum1 = q*q*q;
        dum1 = acos(r/sqrt(dum1));
        r13 = 2.0*sqrt(q);
        double x1r= -term1 + r13*cos(dum1/3.0);
        x2r = -term1 + r13*cos((dum1 + 2.0*M_PI)/3.0);
        double x3r = -term1 + r13*cos((dum1 + 4.0*M_PI)/3.0);
        roots[0]= x1r;
        roots[1] = x2r;
        roots[2]= x3r;
        return 3;
}  //End of cubicSolve




//============================ compute times at which there is max or min vel, acc, jrk ===============
void BezierQuinticSegment::compute_maxmin_times(){
//    std::cout<<">>>>>>> compute_maxmin_times: t_star, t_end:  " << t_start_ << ",   "<< t_end_<< std::endl;
//    std::cout<<">>compute_maxmin_times: coef_: \n" <<"P0-P5: " <<coef_[0] <<",  " <<coef_[1]<<",  " <<coef_[2]<<",  " <<coef_[3]<<",  " <<coef_[4]<<",  " <<coef_[5]<<std::endl;
    compute_coef();
    double A=0, B=0, C=0, D=0, a=0, b=0, c=0, d=0, disc=0;
    double t1=t_start_, t2=t_end_;
    double P0=coef_[0], P1=coef_[1], P2=coef_[2], P3=coef_[3], P4=coef_[4], P5=coef_[5];
    double r1=0, r2=0;
    std::vector<double> rts;
    int n_rts=0;
    /*======================================== snp ==0 ========================================
     * snp = 120*P1*(t - t1) - 60*P0*(2*t - 2*t2) + 240*P1*(2*t - 2*t2) - 240*P2*(2*t - 2*t1) - 360*P2*(2*t - 2*t2) + 360*P3*(2*t - 2*t1) + 240*P3*(2*t - 2*t2) - 240*P4*(2*t - 2*t1) + 60*P5*(2*t - 2*t1) - 24*P4*(5*t - 5*t2)
     * snp =at+b,  a = 600*P1 - 120*P0 - 1200*P2 + 1200*P3 - 600*P4 + 120*P5
     * b= 120*P0*t2 - 120*P1*t1 - 480*P1*t2 + 480*P2*t1 + 720*P2*t2 - 720*P3*t1 - 480*P3*t2 + 480*P4*t1 + 120*P4*t2 - 120*P5*t1
    ===========================================================================================*/
    a = 600*P1 - 120*P0 - 1200*P2 + 1200*P3 - 600*P4 + 120*P5;
    b = 120*P0*t2 - 120*P1*t1 - 480*P1*t2 + 480*P2*t1 + 720*P2*t2 - 720*P3*t1 - 480*P3*t2 + 480*P4*t1 + 120*P4*t2 - 120*P5*t1;
    if(a==0.000000)
        tj_= 0; //no roots
    else
        tj_= -b/a;

    /*======================================== jrk ==0 ========================================
     * jrk = 180*P1*(t - t2)^2 - 60*P0*(t - t2)^2 - 60*P2*(t - t1)^2 - 180*P2*(t - t2)^2 + 180*P3*(t - t1)^2 + 60*P3*(t - t2)^2 - 180*P4*(t - t1)^2 + 60*P5*(t - t1)^2 + 60*P1*(2*t - 2*t2)*(t - t1) - 90*P2*(2*t - 2*t1)*(2*t - 2*t2) + 90*P3*(2*t - 2*t1)*(2*t - 2*t2) - 12*P4*(2*t - 2*t1)*(5*t - 5*t2)
     * jrk = at^2+bt+c
     * A=  180*P1 -  60*P0 - 180*P2 + 60*P3;
     * B= -60*P2  + 180*P3 - 180*P4 + 60*P5;
     * C=  120*P1 - 360*P2 + 360*P3 - 120*P4;
     * a= A+B+C;
     * b= - (2*t2*A + 2*t1*B + t1*C + t2*C);
     * c= (A*t2*t2 + B*t1*t1 + C*t1*t2);
    ===========================================================================================*/
    A=  180*P1 -  60*P0 - 180*P2 + 60*P3;
    B= -60*P2  + 180*P3 - 180*P4 + 60*P5;
    C=  120*P1 - 360*P2 + 360*P3 - 120*P4;
    a= A+B+C;
    b= - (2*t2*A + 2*t1*B + t1*C + t2*C);
    c= (A*t2*t2 + B*t1*t1 + C*t1*t2);
    n_rts= quad_eq_real_root (a, b, c, r1,r2);
    ta1_ = r1;
    ta2_ = r2;
//    std::cout<<"#acc_minmax: "<< n_rts << "{ "<< ta1_ << ",  "<< ta2_<<"}" <<std::endl;
    /*======================================== acc ==0 ========================================
     * acc = 40*P1*(t - t2)^3 - 20*P0*(t - t2)^3 - 20*P2*(t - t2)^3 + 20*P3*(t - t1)^3 - 40*P4*(t - t1)^3 + 20*P5*(t - t1)^3 + 60*P1*(t - t1)*(t - t2)^2 - 60*P2*(2*t - 2*t1)*(t - t2)^2 - 30*P2*(2*t - 2*t2)*(t - t1)^2 + 30*P3*(2*t - 2*t1)*(t - t2)^2 + 60*P3*(2*t - 2*t2)*(t - t1)^2 - 12*P4*(5*t - 5*t2)*(t - t1)^2
     * acc = a*t^3 + b*t^2 + c*t + d
     * A=  40*P1 -  20*P0 - 20*P2 ;
     * B= 20*P3  - 40*P4 + 20*P5;
     * C=  60*P1 - 120*P2 + 60*P3 ;
     * D=  -60*P2 + 120*P3 - 60*P4 ;
     * a= (A+B+C+D);
     * b= -(3*t2*A + 3*t1*B + (2*t2+t1)*C + (2*t1+t2)*D );
     * c= (3*pow(t2, 2)*A + 3*pow(t1, 2)*B + (pow(t2, 2)+2*t1*t2)*C +  (pow(t1, 2)+2*t1*t2)*D );
     * d= -(pow(t2,3)*A + pow(t1,3)*B +  t1*pow(t2, 2)*C  + pow(t1, 2)*t2*D   );
    ===========================================================================================*/
    A=  40*P1 -  20*P0 - 20*P2 ;
    B= 20*P3  - 40*P4 + 20*P5;
    C=  60*P1 - 120*P2 + 60*P3 ;
    D=  -60*P2 + 120*P3 - 60*P4 ;
    a= (A+B+C+D);
    b= -(3*t2*A + 3*t1*B + (2*t2+t1)*C + (2*t1+t2)*D );
    c= (3*pow(t2, 2)*A + 3*pow(t1, 2)*B + (pow(t2, 2)+2*t1*t2)*C +  (pow(t1, 2)+2*t1*t2)*D );
    d= -(pow(t2,3)*A + pow(t1,3)*B +  t1*pow(t2, 2)*C  + pow(t1, 2)*t2*D   );
    if(a==0.000000){
        tv1_ = 0;
        n_rts= quad_eq_real_root ( b, c, d, r1,r2);
        tv2_ = r1;
        tv3_ = r2;
//        std::cout<<"#acc_n_rts: "<< n_rts << "{ "<< tv1_ << ",  "<< tv2_<< ",  "<< tv3_<<"}" <<std::endl;
    }else {
        n_rts= cubic_eq_real_root (a, b, c, d, rts);
        tv1_ = rts[0];
        tv2_ = rts[1];
        tv3_ = rts[2];
//        std::cout<<"#acc_n_rts: "<< n_rts << "{ "<< tv1_ << ",  "<< tv2_<<"}" << ",  "<< tv3_ <<std::endl;
    }

    check_maxmin_times_interval ();
//    std::cout<<"#jrk_minmax: "<< tj_ <<std::endl;
//    std::cout<<"#acc_minmax: "<< ta1_ << ",  "<< ta2_<<std::endl;
//    std::cout<<"#vel_minmax: "<< tv1_ << ",  "<< tv2_ << ",  "<< tv3_<<std::endl;
//    std::cout<<"#start_end : "<< t_start_ << ",  "<< t_end_ <<std::endl;

//  this give like nested endless looping error   std::cout<<"#monotonic : "<< check_monotonic() <<std::endl;

}



//====================== check if max_min time are inside the segment interval ==============
void BezierQuinticSegment::check_maxmin_times_interval(){
    if( tj_ < t_start_)
        tj_ = t_start_;
    if( tj_ > t_end_)
        tj_ = t_end_;

    if( ta1_ < t_start_)
        ta1_ = t_start_;
    if( ta1_ > t_end_)
        ta1_ = t_end_;

    if( ta2_ < t_start_)
        ta2_ = t_start_;
    if( ta2_ > t_end_)
        ta2_ = t_end_;

    if( tv1_ < t_start_)
        tv1_ = t_start_;
    if( tv1_ > t_end_)
        tv1_ = t_end_;

    if( tv2_ < t_start_)
        tv2_ = t_start_;
    if( tv2_ > t_end_)
        tv2_ = t_end_;

    if( tv3_ < t_start_)
        tv3_ = t_start_;
    if( tv3_ > t_end_)
        tv3_ = t_end_;
}




//===================================sample segment at time t =========================
// this are used only for plotting during offline testing
void BezierQuinticSegment::sample_segment(double t, std::vector<double> &state) {
    double t1=t_start_, t2=t_end_;
    double P0=coef_[0], P1=coef_[1], P2=coef_[2], P3=coef_[3], P4=coef_[4], P5=coef_[5];
    state.resize(4); //pos, vel, acc, jrk
    state[0] = P5*pow( (t-t1), 5) - P0*pow( (t-t2), 5) + 5*P1*(t - t1)*pow( (t-t2), 4) - P4*(5*t - 5*t2)*pow( (t-t1), 4) - 10*P2*pow( (t-t1), 2)*pow( (t-t2), 3) + 10*P3*pow( (t-t1), 3)*pow( (t-t2), 2);
    state[1] = 5*P1*pow( (t-t2), 4) - 5*P0*pow( (t-t2), 4) - 5*P4*pow( (t-t1), 4) + 5*P5*pow( (t-t1), 4) + 20*P1*(t - t1)*pow( (t-t2), 3) - 10*P2*(2*t - 2*t1)*pow( (t-t2), 3) + 10*P3*(2*t - 2*t2)*pow( (t-t1), 3) - 4*P4*(5*t - 5*t2)*pow( (t-t1), 3) - 30*P2*pow( (t-t1), 2)*pow( (t-t2), 2) + 30*P3*pow( (t-t1), 2)*pow( (t-t2), 2);
    state[2] = 40*P1*pow( (t-t2), 3) - 20*P0*pow( (t-t2), 3) - 20*P2*pow( (t-t2), 3) + 20*P3*pow( (t-t1), 3) - 40*P4*pow( (t-t1), 3) + 20*P5*pow( (t-t1), 3) + 60*P1*(t - t1)*pow( (t-t2), 2) - 60*P2*(2*t - 2*t1)*pow( (t-t2), 2) - 30*P2*(2*t - 2*t2)*pow( (t-t1), 2) + 30*P3*(2*t - 2*t1)*pow( (t-t2), 2) + 60*P3*(2*t - 2*t2)*pow( (t-t1), 2) - 12*P4*(5*t - 5*t2)*pow( (t-t1), 2);
    state[3] = 180*P1*pow( (t-t2), 2) - 60*P0*pow( (t-t2), 2) - 60*P2*pow( (t-t1), 2) - 180*P2*pow( (t-t2), 2) + 180*P3*pow( (t-t1), 2) + 60*P3*pow( (t-t2), 2) - 180*P4*pow( (t-t1), 2) + 60*P5*pow( (t-t1), 2) + 60*P1*(2*t - 2*t2)*(t - t1) - 90*P2*(2*t - 2*t1)*(2*t - 2*t2) + 90*P3*(2*t - 2*t1)*(2*t - 2*t2) - 12*P4*(2*t - 2*t1)*(5*t - 5*t2);
}
void BezierQuinticSegment::segment_states(const double frq, std::vector<double> &T_vec, std::vector<double> &POS, std::vector<double> &VEL, std::vector<double> &ACC, std::vector<double> &JRK ){
    double t=t_start_;
    std::vector<double> state;
    while(t<t_end_){
        sample_segment(t, state);
        T_vec.push_back(t);
        POS.push_back(state[0]);
        VEL.push_back(state[1]);
        ACC.push_back(state[2]);
        JRK.push_back(state[3]);
        t+=1/frq;
    }
}






//======================== print attribute for checking behavoir ==========================
void BezierQuinticSegment::print_attributes(){
    std::cout<<"======================== segment attributes ========================="<<std::endl;

    std::cout<<"#########coef_: \n" <<"# P0-P5: " <<coef_[0] <<",  " <<coef_[1]<<",  " <<coef_[2]<<",  " <<coef_[3]<<",  " <<coef_[4]<<",  " <<coef_[5]<<std::endl;
    std::cout<<"#########limits: \n" <<"# pos: " <<max_pos_ <<"  #vel: " <<max_vel_<<"  #acc: " <<max_acc_<<"  #jrk: " <<max_jrk_<<std::endl;

    std::cout<<"#########times: " << std::endl;
    std::cout<<"# start_end : " <<t_start_ <<",  " <<t_end_<<" ==>   duration: " <<duration_<<std::endl;
    std::cout<<"# jrk_minmax: "<< tj_ <<std::endl;
    std::cout<<"# acc_minmax: "<< ta1_ << ",  "<< ta2_<<std::endl;
    std::cout<<"# vel_minmax: "<< tv1_ << ",  "<< tv2_ << ",  "<< tv3_<<std::endl;

    std::cout<<"#########start_state: \n" <<"# pos0: " <<start_state_[0] <<"  #vel0: " <<start_state_[1]<<"  #acc0: " <<start_state_[2]<<std::endl;
    std::cout<<"#########start_state: \n" <<"# pos1: " <<end_state_[0] <<"  #vel1: " <<end_state_[1]<<"  #acc1: " <<end_state_[2]<<std::endl;

    std::cout<<"######iterations: "<<iter_<<"     ######monotonic: "<<check_monotonic()<<std::endl;

    std::cout<<"=========================================================================="<<std::endl;

}



