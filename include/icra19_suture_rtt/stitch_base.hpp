#ifndef _STITCH_BASE_HPP_
#define _STITCH_BASE_HPP_


#include <star_rtt/trajectory.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
//#include <iostream>
//#include "stitch_helper.hpp"

#define RELEASE_MM 0.001
geometry_msgs::Vector3 zero3();

class StitchBase {
  
public:
  
  enum State { 
    IDLE       = -1,
    START      = 0,
    APPROACH   = 1,
    CONTACT    = 2,
    SLIDE      = 3,
    JOINTSTATE = 4,
    STITCH     = 5,
    LIFT       = 6,
    TENSION    = 7,
    RELEASE    = 8,
    FINISHED   = 9, 
    HOVER      = 10,
    STITCH_FIRE= 11,
    SLIDE_ROT  = 12
  };
  
  //! Constructor
  StitchBase( const KDL::Frame& Rts, 
	      double f, 
	      double d, 
	      double t = 4.0 );
  
  //! Deconstructor
  virtual ~StitchBase();

  // virtual evaluate with accept_stitch read from stitch_configure
  virtual State Evaluate( KDL::Frame& Rt,
			  KDL::Twist& vw,
			  KDL::JntArray& q,
			  KDL::JntArray& qd,
			  const KDL::Vector& f = KDL::Vector::Zero(),
			  bool accept_stitch = false,          
			  bool repeat_stitch = false,
			  const geometry_msgs::Vector3& offset_vector  = zero3(),
			  const geometry_msgs::Vector3& contact_vector = zero3(),
			  const geometry_msgs::Vector3& contact_force  = zero3(),
			  const geometry_msgs::Vector3& tension_vector = zero3(),
			  const geometry_msgs::Vector3& tension_forice = zero3(),
			  bool accept_tension = false ) = 0;
  
  // Virtual Functions
  virtual void Start( KDL::Frame& Rt ) = 0;
  virtual void Approach() = 0;
  //virtual void Contact() = 0;
  //virtual void Stitch()=0;
  //virtual void Lift()=0;
  //virtual void Tension()=0;
  //virtual void Release()=0;
  //virtual void Finished()=0;
  virtual void Hover() = 0;
  
  KDL::Vector StitchXYZ() const {
    return Rts.p;
    //return trajectory.TargetXYZ(); 
  }
  
protected:
  
  State state;
  std::string name;
  
  Trajectory trajectory;
  
  KDL::Frame Rts;
  KDL::Frame Rts_offset; // New as of 12-29-2014
  
  KDL::Vector f_ref;
  double tension_f;
  double tension_d;
  
  ros::Duration stitch_duration;
  ros::Time stitch_start_time;

  void ForwardKinematics( const KDL::JntArray& q, KDL::Frame& Rt );
  void InverseKinematics( const KDL::Frame& Rt, KDL::JntArray& qold );
  
  void NewTarget( const KDL::Frame& kdlRt );
  void NewTarget( const KDL::JntArray& q ); 
  
  
  Trajectory::Errno EvaluateTrajectory( KDL::Frame& kdlRt );
  Trajectory::Errno EvaluateTrajectory( KDL::JntArray& kdlq ); 
 

  void print( const KDL::Frame& Rt, const std::string& s="" );
 
  
};

#endif
