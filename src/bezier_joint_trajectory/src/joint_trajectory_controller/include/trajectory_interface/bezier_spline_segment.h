///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
// Copyright (c) 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Mahmoud Ali
/// this class is based on the quitnic_spline_segment template written by Adolfo Rodriguez Tsouroukdissian, Stuart Glaser, Mrinal Kalakrishnan

#ifndef TRAJECTORY_INTERFACE_QUINTIC_SPLINE_SEGMENT_H
#define TRAJECTORY_INTERFACE_QUINTIC_SPLINE_SEGMENT_H

#include <array>
#include <iterator>
#include <stdexcept>
#include <trajectory_interface/pos_vel_acc_state.h>
#include<iostream>


namespace trajectory_interface
{

/**
 * \brief Class representing a multi-dimensional quintic spline segment with a start and end time.
 *
 * \tparam ScalarType Scalar type
 */
template<class ScalarType>
class BezierSplineSegment
{
public:
  typedef ScalarType             Scalar;
  typedef Scalar                 Time;
  typedef PosVelAccState<Scalar> State;

  /**
   * \brief Creates an empty segment.
   *
   * \note Calling <tt> size() </tt> on an empty segment will yield zero, and sampling it will yield a state with empty
   * data.
   */
  BezierSplineSegment()
    : coefs_(),
      duration_(static_cast<Scalar>(0)),
      start_time_(static_cast<Scalar>(0))
  {}

  /**
   * \brief Construct segment from start and end states (boundary conditions).
   *
   * Please refer to the \ref init method documentation for the description of each parameter and the exceptions that
   * can be thrown.
   */
  BezierSplineSegment(const Time&  start_time,
                       const State& start_state,
                       const Time&  end_time,
                       const State& end_state)
  {
    init(start_time, start_state, end_time, end_state);
  }

  /**
   * \brief Initialize segment from start and end states (boundary conditions).
   *
   * The start and end states need not necessarily be specified all the way to the acceleration level:
   * - If only \b positions are specified, linear interpolation will be used.
   * - If \b positions and \b velocities are specified, a cubic spline will be used.
   * - If \b positions, \b velocities and \b accelerations are specified, a quintic spline will be used.
   *
   * \note If start and end states have different specifications
   * (eg. start is positon-only, end is position-velocity), the lowest common specification will be used
   * (position-only in the example).
   *
   * \param start_time Time at which the segment state equals \p start_state.
   * \param start_state State at \p start_time.
   * \param end_time Time at which the segment state equals \p end_state.
   * \param end_state State at time \p end_time.
   *
   * \throw std::invalid_argument If the \p end_time is earlier than \p start_time or if one of the states is
   * uninitialized.
   */
  void init(const Time&  start_time,
            const State& start_state,
            const Time&  end_time,
            const State& end_state);

  /**
   * \brief Sample the segment at a specified time.
   *
   * \note Within the <tt>[start_time, end_time]</tt> interval, spline interpolation takes place, outside it this method
   * will output the start/end states with zero velocity and acceleration.
   *
   * \param[in] time Where the segment is to be sampled.
   * \param[out] state Segment state at \p time.
   */
  void sample(const Time& time, State& state) const
  {
    // Resize state data. Should be a no-op if appropriately sized
    state.position.resize(coefs_.size());
    state.velocity.resize(coefs_.size());
    state.acceleration.resize(coefs_.size());

    // Sample each dimension
    typedef typename std::vector<SplineCoefficients>::const_iterator ConstIterator;
    for(ConstIterator coefs_it = coefs_.begin(); coefs_it != coefs_.end(); ++coefs_it)
    {
      const typename std::vector<Scalar>::size_type id = std::distance(coefs_.begin(), coefs_it);
      sampleWithTimeBounds(*coefs_it,
                           start_time_, (start_time_+duration_), time,
                           state.position[id], state.velocity[id], state.acceleration[id]);
    }
  }





  /** \return Segment start time. */
  Time startTime() const {return start_time_;}

  /** \return Segment end time. */
  Time endTime() const {return start_time_ + duration_;}

  /** \return Segment size (dimension). */
  unsigned int size() const {return coefs_.size();}

private:
  typedef std::array<Scalar, 6> SplineCoefficients;

  /** Coefficients represent a quintic polynomial like so:
   *
   * <tt> coefs_[0] + coefs_[1]*x + coefs_[2]*x^2 + coefs_[3]*x^3 + coefs_[4]*x^4 + coefs_[5]*x^5 </tt>
   */
  std::vector<SplineCoefficients> coefs_;
  Time duration_;
  Time start_time_;

  // These methods are borrowed from the previous controller's implementation
  // TODO: Clean their implementation, use the Horner algorithm for more numerically stable polynomial evaluation
  static void generatePowers(int n, const Scalar& x, Scalar* powers);
  static Scalar pow( const Scalar& x, int n);

  static void computeCoefficients(const Scalar& start_pos,
                                  const Scalar& end_pos,
                                  const Scalar& t1_start,  const Scalar& t2_end,
                                  SplineCoefficients& coefficients);

  static void computeCoefficients(const Scalar& start_pos, const Scalar& start_vel,
                                  const Scalar& end_pos,   const Scalar& end_vel,
                                  const Scalar& t1_start,  const Scalar& t2_end,
                                  SplineCoefficients& coefficients);

  static void computeCoefficients(const Scalar& start_pos, const Scalar& start_vel, const Scalar& start_acc,
                                  const Scalar& end_pos,   const Scalar& end_vel,   const Scalar& end_acc,
                                  const Scalar& t1, const Scalar& t2,
                                  SplineCoefficients& coefficients);



  static  void sample(const SplineCoefficients& coefficients, const Scalar& t1, const Scalar& t2, const Scalar& time,
                     Scalar& position, Scalar& velocity, Scalar& acceleration);


  static void sampleWithTimeBounds(const SplineCoefficients& coefficients, const Scalar& t1, const Scalar& t2, const Scalar& time,
                                Scalar& position, Scalar& velocity, Scalar& acceleration);
};

template<class ScalarType>
void BezierSplineSegment<ScalarType>::init(const Time&  start_time,
                                            const State& start_state,
                                            const Time&  end_time,
                                            const State& end_state)
{
  // Preconditions
  if (end_time < start_time)
  {
    throw(std::invalid_argument("BezierSplineSegment can't be constructed: end_time < start_time."));
  }
  if (start_state.position.empty() || end_state.position.empty())
  {
    throw(std::invalid_argument("BezierSplineSegment can't be constructed: Endpoint positions can't be empty."));
  }
  if (start_state.position.size() != end_state.position.size())
  {
    throw(std::invalid_argument("BezierSplineSegment can't be constructed: Endpoint positions size mismatch."));
  }

  const unsigned int dim = start_state.position.size();
  const bool has_velocity     = !start_state.velocity.empty()     && !end_state.velocity.empty();
  const bool has_acceleration = !start_state.acceleration.empty() && !end_state.acceleration.empty();

  if (has_velocity && dim != start_state.velocity.size())
  {
    throw(std::invalid_argument("BezierSplineSegment can't be constructed: Start state velocity size mismatch."));
  }
  if (has_velocity && dim != end_state.velocity.size())
  {
    throw(std::invalid_argument("BezierSplineSegment can't be constructed: End state velocity size mismatch."));
  }
  if (has_acceleration && dim!= start_state.acceleration.size())
  {
    throw(std::invalid_argument("BezierSplineSegment can't be constructed: Start state acceleration size mismatch."));
  }
  if (has_acceleration && dim != end_state.acceleration.size())
  {
    throw(std::invalid_argument("BezierSplineSegment can't be constructed: End state acceleratios size mismatch."));
  }

  // Time data
  start_time_ = start_time;
  duration_   = end_time - start_time;


//  std::cout<<"initSegment, start_time, duration: "<< start_time_ <<"  ,  "<< duration_ <<std::endl;
  // Spline coefficients
  coefs_.resize(dim);

  typedef typename std::vector<SplineCoefficients>::iterator Iterator;
  if (!has_velocity)
  {
    // Linear interpolation
//      std::cout<<"Linear: t_start, dur, t_end: "<< start_time_ <<"  ,  "<< duration_ <<"  ,  "<< start_time_ + duration_<<std::endl;
    for(Iterator coefs_it = coefs_.begin(); coefs_it != coefs_.end(); ++coefs_it)
    {
      const typename std::vector<Scalar>::size_type id = std::distance(coefs_.begin(), coefs_it);
//      std::cout<<"Linear: start_pos,  end_pos: "<< start_state.position[id] <<"  ,  "<< end_state.position[id]<<std::endl;
      computeCoefficients(start_state.position[id],
                          end_state.position[id],
                          start_time_, start_time_ + duration_,
                          *coefs_it);
    }
  }
  else if (!has_acceleration)
  {
    // Cubic interpolation
//      std::cout<<"Cubic: t_start, dur, t_end: "<< start_time_ <<"  ,  "<< duration_ <<"  ,  "<< start_time_ + duration_<<std::endl;
    for(Iterator coefs_it = coefs_.begin(); coefs_it != coefs_.end(); ++coefs_it)
    {
      const typename std::vector<Scalar>::size_type id = std::distance(coefs_.begin(), coefs_it);
//      std::cout<<"Cubic: start_pos,  end_pos: "<< start_state.position[id] <<"  ,  "<< end_state.position[id]<<std::endl;
      computeCoefficients(start_state.position[id], start_state.velocity[id],
                          end_state.position[id],   end_state.velocity[id],
                          start_time_, start_time_ + duration_,
                          *coefs_it);
    }
  }
  else
  {
    // Quintic interpolation
//      std::cout<<"Quintic: t_start, dur, t_end: "<< start_time_ <<"  ,  "<< duration_ <<"  ,  "<< start_time_ + duration_<<std::endl;
    for(Iterator coefs_it = coefs_.begin(); coefs_it != coefs_.end(); ++coefs_it)
    {
      const typename std::vector<Scalar>::size_type id = std::distance(coefs_.begin(), coefs_it);
//      std::cout<<"Quintic: start_pos,  end_pos: "<< start_state.position[id] <<"  ,  "<< end_state.position[id]<<std::endl;
      computeCoefficients(start_state.position[id], start_state.velocity[id], start_state.acceleration[id],
                          end_state.position[id],   end_state.velocity[id],   end_state.acceleration[id],
                          start_time_, start_time_ + duration_,
                          *coefs_it);
    }
  }
}


template<class ScalarType>
inline void BezierSplineSegment<ScalarType>::generatePowers(int n, const Scalar& x, Scalar* powers)
{
  powers[0] = 1.0;
  for (int i=1; i<=n; ++i)
  {
    powers[i] = powers[i-1]*x;
  }
}

template<class ScalarType>
inline ScalarType BezierSplineSegment<ScalarType>::pow( const Scalar& x, int n)
{
  Scalar power=1;
  for (int i=1; i<=n; ++i)
  {
    power *= x;
  }
  return power;
}

//linear interpolation --------------------------------------------------------------
template<class ScalarType>
void BezierSplineSegment<ScalarType>::
computeCoefficients(const Scalar& start_pos,
                    const Scalar& end_pos,
                    const Scalar& t1_start,  const Scalar& t2_end,
                    SplineCoefficients& coefficients)
{
    double t1=t1_start,  t2=t2_end;
    double start_vel =0, end_vel=0, start_acc=0, end_acc=0;
//    std::cout<<"Linear: start_vel,  end_vel: "<< start_vel <<"  ,  "<< end_vel<<std::endl;
//    std::cout<<"Linear: start_acc,  end_acc: "<< start_acc <<"  ,  "<< end_acc<<std::endl;

    if ((t2-t1) == 0.0)
    {
      coefficients[0] = start_pos;
      coefficients[1] = start_vel;
      coefficients[2] = 0.5*start_acc;
      coefficients[3] = 0.0;
      coefficients[4] = 0.0;
      coefficients[5] = 0.0;
    }
    else
    {
      double p0=start_pos, v0=start_vel, a0=start_acc, pn=end_pos, vn=end_vel, an=end_acc;
      coefficients[0] = -p0/pow((t1-t2), 5 ) ;
      coefficients[1] = -(5*p0 - t1*v0 + t2*v0)/(5*pow((t1-t2), 5 ));
      coefficients[2] = -(a0*t1*t1 - 2*a0*t1*t2 - 8*v0*t1 + a0*t2*t2 + 8*v0*t2 + 20*p0)/(20*pow((t1-t2), 5 ));
      coefficients[3] = -(an*t1*t1 - 2*an*t1*t2 + 8*vn*t1 + an*t2*t2 - 8*vn*t2 + 20*pn)/(20*pow((t1-t2), 5 ));
      coefficients[4] = -(5*pn + t1*vn - t2*vn)/(5*pow((t1-t2), 5 ));
      coefficients[5] = -pn/pow((t1-t2), 5 );
    }

}


//cubic interpolation --------------------------------------------------------------
template<class ScalarType>
void BezierSplineSegment<ScalarType>::
computeCoefficients(const Scalar& start_pos, const Scalar& start_vel,
                    const Scalar& end_pos,   const Scalar& end_vel,
                    const Scalar& t1_start,  const Scalar& t2_end,
                    SplineCoefficients& coefficients)
{
    double t1=t1_start,  t2=t2_end;
    double start_acc=0, end_acc=0;
//    std::cout<<"Cubic: start_vel,  end_vel: "<< start_vel <<"  ,  "<< end_vel<<std::endl;
//    std::cout<<"Cubic: start_acc,  end_acc: "<< start_acc <<"  ,  "<< end_acc<<std::endl;
    if ((t2-t1) == 0.0)
    {
      coefficients[0] = start_pos;
      coefficients[1] = start_vel;
      coefficients[2] = 0.5*start_acc;
      coefficients[3] = 0.0;
      coefficients[4] = 0.0;
      coefficients[5] = 0.0;
    }
    else
    {
      double p0=start_pos, v0=start_vel, a0=start_acc, pn=end_pos, vn=end_vel, an=end_acc;

      coefficients[0] = -p0/pow((t1-t2), 5 ) ;
      coefficients[1] = -(5*p0 - t1*v0 + t2*v0)/(5*pow((t1-t2), 5 ));
      coefficients[2] = -(a0*t1*t1 - 2*a0*t1*t2 - 8*v0*t1 + a0*t2*t2 + 8*v0*t2 + 20*p0)/(20*pow((t1-t2), 5 ));
      coefficients[3] = -(an*t1*t1 - 2*an*t1*t2 + 8*vn*t1 + an*t2*t2 - 8*vn*t2 + 20*pn)/(20*pow((t1-t2), 5 ));
      coefficients[4] = -(5*pn + t1*vn - t2*vn)/(5*pow((t1-t2), 5 ));
      coefficients[5] = -pn/pow((t1-t2), 5 );
    }

}


//quintic interpolation --------------------------------------------------------------
template<class ScalarType>
void BezierSplineSegment<ScalarType>::
computeCoefficients(const Scalar& start_pos, const Scalar& start_vel, const Scalar& start_acc,
                    const Scalar& end_pos,   const Scalar& end_vel,   const Scalar& end_acc,
                    const Scalar& t1_start, const Scalar& t2_end,
                    SplineCoefficients& coefficients)
{
  double t1=t1_start,  t2=t2_end;
//  std::cout<<"Quintic: start_vel,  end_vel: "<< start_vel <<"  ,  "<< end_vel<<std::endl;
//  std::cout<<"Quintic: start_acc,  end_acc: "<< start_acc  <<"  ,  "<< end_acc<<std::endl;
  if ((t2-t1) == 0.0)
  {
    coefficients[0] = start_pos;
    coefficients[1] = start_vel;
    coefficients[2] = 0.5*start_acc;
    coefficients[3] = 0.0;
    coefficients[4] = 0.0;
    coefficients[5] = 0.0;
  }
  else
  {      
    double p0=start_pos, v0=start_vel, a0=start_acc, pn=end_pos, vn=end_vel, an=end_acc;
    coefficients[0] = -p0/pow((t1-t2), 5 ) ;
    coefficients[1] = -(5*p0 - t1*v0 + t2*v0)/(5*pow((t1-t2), 5 ));
    coefficients[2] = -(a0*t1*t1 - 2*a0*t1*t2 - 8*v0*t1 + a0*t2*t2 + 8*v0*t2 + 20*p0)/(20*pow((t1-t2), 5 ));
    coefficients[3] = -(an*t1*t1 - 2*an*t1*t2 + 8*vn*t1 + an*t2*t2 - 8*vn*t2 + 20*pn)/(20*pow((t1-t2), 5 ));
    coefficients[4] = -(5*pn + t1*vn - t2*vn)/(5*pow((t1-t2), 5 ));
    coefficients[5] = -pn/pow((t1-t2), 5 );
  }

//  std::cout<<"initSegment, t1, t2: "<< t1 <<"  ,  "<< t2 <<std::endl;
//  std::cout<<"Coef: "<< coefficients[0] <<",  "<< coefficients[1] <<",  "<< coefficients[2] <<",  "<< coefficients[3] <<",  "<< coefficients[4] <<std::endl;

}





//sample --------------------------------------------------------------
template<class ScalarType>
void BezierSplineSegment<ScalarType>::
sample(const SplineCoefficients& coefficients, const Scalar& t1_start, const Scalar& t2_end, const Scalar& time,
       Scalar& position, Scalar& velocity, Scalar& acceleration)
{
 // create powers of time:
  // this state equations are correct only with bezier coef_, they are not correspond to cubic or linear coef_
  double t=time;
  double t1=t1_start,  t2=t2_end;
//  std::cout<<"t1, time, t2:"<< t1 <<"  ,"<< t <<"  ,"<< t2 << std::endl;
  double P0=coefficients[0], P1=coefficients[1], P2=coefficients[2];
  double P3=coefficients[3], P4=coefficients[4], P5=coefficients[5];
 if(t1-t2==0.0){
      position = P0;
      velocity = P1;
      acceleration = 2*P2;
 }
// else if (time <= t1_start){
//      position = -P0*pow((t1 - t2), 5);
//      velocity = P1;
//      acceleration = 2*P2;
//  }else if(time >= t2_end){
//      position = -P5*pow((t1 - t2), 5);
// }
 else{
     position = P5*pow( (t-t1), 5) - P0*pow( (t-t2), 5) + 5*P1*(t - t1)*pow( (t-t2), 4) - P4*(5*t - 5*t2)*pow( (t-t1), 4) - 10*P2*pow( (t-t1), 2)*pow( (t-t2), 3) + 10*P3*pow( (t-t1), 3)*pow( (t-t2), 2);
     velocity = 5*P1*pow( (t-t2), 4) - 5*P0*pow( (t-t2), 4) - 5*P4*pow( (t-t1), 4) + 5*P5*pow( (t-t1), 4) + 20*P1*(t - t1)*pow( (t-t2), 3) - 10*P2*(2*t - 2*t1)*pow( (t-t2), 3) + 10*P3*(2*t - 2*t2)*pow( (t-t1), 3) - 4*P4*(5*t - 5*t2)*pow( (t-t1), 3) - 30*P2*pow( (t-t1), 2)*pow( (t-t2), 2) + 30*P3*pow( (t-t1), 2)*pow( (t-t2), 2);
     acceleration = 40*P1*pow( (t-t2), 3) - 20*P0*pow( (t-t2), 3) - 20*P2*pow( (t-t2), 3) + 20*P3*pow( (t-t1), 3) - 40*P4*pow( (t-t1), 3) + 20*P5*pow( (t-t1), 3) + 60*P1*(t - t1)*pow( (t-t2), 2) - 60*P2*(2*t - 2*t1)*pow( (t-t2), 2) - 30*P2*(2*t - 2*t2)*pow( (t-t1), 2) + 30*P3*(2*t - 2*t1)*pow( (t-t2), 2) + 60*P3*(2*t - 2*t2)*pow( (t-t1), 2) - 12*P4*(5*t - 5*t2)*pow( (t-t1), 2);
   //jerk = 180*P1*pow( (t-t2), 2) - 60*P0*pow( (t-t2), 2) - 60*P2*pow( (t-t1), 2) - 180*P2*pow( (t-t2), 2) + 180*P3*pow( (t-t1), 2) + 60*P3*pow( (t-t2), 2) - 180*P4*pow( (t-t1), 2) + 60*P5*pow( (t-t1), 2) + 60*P1*(2*t - 2*t2)*(t - t1) - 90*P2*(2*t - 2*t1)*(2*t - 2*t2) + 90*P3*(2*t - 2*t1)*(2*t - 2*t2) - 12*P4*(2*t - 2*t1)*(5*t - 5*t2);
//     std::cout<<"time, pos, vel, acc: "<< time << ",  " <<position << ",  " <<velocity << ",  " <<acceleration <<std::endl ;

 }


}

template<class ScalarType>
void BezierSplineSegment<ScalarType>::
sampleWithTimeBounds(const SplineCoefficients& coefficients, const Scalar& t1, const Scalar& t2, const Scalar& time,
                     Scalar& position, Scalar& velocity, Scalar& acceleration)
{
//    std::cout<<"time: "<< time<<std::endl ;

  if (time < t1)
  {
    Scalar _;
    sample(coefficients, t1, t2,  t1, position, _, _);
    velocity = 0;
    acceleration = 0;
//    std::cout<<"##Before seg: time, pos, vel, acc: "<< time << ",  " <<position << ",  " <<velocity << ",  " <<acceleration <<std::endl ;

  }
  else if (time > t2 )
  {
    Scalar _;
    sample(coefficients, t1, t2,  t2 , position, _, _);
    velocity = 0;
    acceleration = 0;
//    std::cout<<"##After seg: time, pos, vel, acc: "<< time << ",  " <<position << ",  " <<velocity << ",  " <<acceleration <<std::endl ;

  }
  else
  {
    sample(coefficients, t1, t2, time,
           position, velocity, acceleration);
//    std::cout<<"##Before seg: time, pos, vel, acc: "<< time << ",  " <<position << ",  " <<velocity << ",  " <<acceleration <<std::endl ;

  }


//    if (time < 0)
//    {
//      Scalar _;
//      sample(coefficients, 0, t2-t1,  0, position, _, _);
//      velocity = 0;
//      acceleration = 0;
//    }
//    else if (time > t2 )
//    {
//      Scalar _;
//      sample(coefficients, 0, t2-t1,  t2-t1 , position, _, _);
//      velocity = 0;
//      acceleration = 0;
//    }
//    else
//    {
//      sample(coefficients, 0, t2-t1, time,
//             position, velocity, acceleration);
//    }





}







} // namespace

#endif // header guard
