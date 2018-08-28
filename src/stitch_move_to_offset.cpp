#include <icra19_suture_rtt/stitch_move_to_offset.hpp>
#include <tf_conversions/tf_kdl.h>

// Constructor for a simple move without stitch
StitchMoveToOffset::StitchMoveToOffset( const KDL::Frame& Rts,
					double cf,
					double cd,
					double tf,
					double td ) :
  StitchBase( Rts, tf, td ),
  contact_f( cf ),
  contact_d( cd ) { 
  name = std::string("StitchMoveToOffset"); 
  }

StitchMoveToOffset::~StitchMoveToOffset() {}


//! Implementation of Start here differs from StitchContact
void StitchMoveToOffset::Start( KDL::Frame& Rt) {
  
  // Important: discard planned orientation in Rts!
  Rts.M = Rt.M;
  tf::Transform tfRt;
  tf::transformKDLToTF( Rt, tfRt );
  
  geometry_msgs::Twist msgvw;
  msgvw.linear.x = 0.0; 
  msgvw.linear.y = 0.0;
  msgvw.linear.z = 0.0; 
  msgvw.angular.x = 0.0;
  msgvw.angular.y = 0.0;
  msgvw.angular.z = 0.0;
  
  geometry_msgs::Twist msgvw_max;
  double max_linear_vel = 0.01;
  double max_angular_vel = 0.1;
  msgvw_max.linear.x = max_linear_vel;
  msgvw_max.linear.y = max_linear_vel;
  msgvw_max.linear.z = max_linear_vel;
  msgvw_max.angular.x = max_angular_vel;
  msgvw_max.angular.y = max_angular_vel;
  msgvw_max.angular.z = max_angular_vel;
  
  trajectory.InitializeMode( tfRt, msgvw, msgvw_max );
  // Note that this trajectory is different than the StitchContact and its subclasses:
  NewTarget( Rts );
} 

void StitchMoveToOffset::Approach() {}
void StitchMoveToOffset::Hover() {}

StitchBase::State StitchMoveToOffset::Evaluate( KDL::Frame& Rt,
						KDL::Twist& vw,
						KDL::JntArray& q,
						KDL::JntArray& qd,
						const KDL::Vector& f,
						bool accept_stitch,
						bool repeat_stitch,
						const geometry_msgs::Vector3& offset_vector,
						const geometry_msgs::Vector3& contact_vector, // Not used here
						const geometry_msgs::Vector3& contact_force,  // Not used here
						const geometry_msgs::Vector3& tension_vector, // Not used here
						const geometry_msgs::Vector3& tension_force,  // Not used here
						bool accept_tension                           // Not used here
						) {
  
  StitchBase::State ret_state = IDLE;
  
  switch( state ) {
    
  case START: {
    ret_state = START;
    StitchMoveToOffset::Start( Rt );
    state = APPROACH;
    std::cout << name << " Approach..." << std::endl;
    break;
  }
    
  case APPROACH: {
    ret_state = APPROACH;
    if( EvaluateTrajectory( Rt ) == Trajectory::EXPIRED ){
      state = FINISHED;
      std::cout << name << " Finished..." << std::endl;
    }
    InverseKinematics( Rt, q );
    break;
  }
  case FINISHED: {
    ret_state = FINISHED;
    EvaluateTrajectory( Rt );
    InverseKinematics( Rt, q );
    break;
  }
  default: {
    ROS_INFO( "Default behavior. Switch case shouldn't end up here!" );
    break;
  }
  }
  
  return ret_state; 
  
}

