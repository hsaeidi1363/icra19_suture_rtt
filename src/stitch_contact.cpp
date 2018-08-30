#include <icra19_suture_rtt/stitch_contact.hpp>
#include <tf_conversions/tf_kdl.h>


//! Constructor for a simple stitch with contact forces
StitchContact::StitchContact( const KDL::Frame& Rts,
			      double cf,
			      double cd,
			      double tf,
			      double td ) :
  StitchBase( Rts, tf, td ),
  contact_f( cf ),
  contact_d( cd ) { 
  name = std::string("StitchContact"); 
  }

//! Deconstructor for a simple stitch with contact forces
StitchContact::~StitchContact() {}

//! Pure virtual Start() implemented at this level
void StitchContact::Start( KDL::Frame& Rt) { 
  
  // Important: discard planned orientation in Rts!
  //Rts.M = Rt.M;
  tf::Transform tfRt;
  tf::transformKDLToTF( Rt, tfRt );
  std::cout << "inside Start of stitch contact: " <<Rt.p[0] << " "<<Rt.p[1] << " " <<Rt.p[2]<< std::endl;
  geometry_msgs::Twist msgvw;   
  msgvw.linear.x = 0.0; 
  msgvw.linear.y = 0.0;
  msgvw.linear.z = 0.0; 
  msgvw.angular.x = 0.0;
  msgvw.angular.y = 0.0;
  msgvw.angular.z = 0.0;
  
  geometry_msgs::Twist msgvw_max;
  double max_linear_vel = 0.01;    // was 0.01 
  double max_angular_vel = 0.01;
  msgvw_max.linear.x = max_linear_vel; 
  msgvw_max.linear.y = max_linear_vel;
  msgvw_max.linear.z = max_linear_vel; 
  msgvw_max.angular.x = max_angular_vel;
  msgvw_max.angular.y = max_angular_vel; 
  msgvw_max.angular.z = max_angular_vel;
  
  trajectory.InitializeMode( tfRt, msgvw, msgvw_max );
  double start_z = -0.002;
  NewTarget( Rts * KDL::Frame( KDL::Vector( 0.0, 0.0, start_z ) ) );
}

/*
void StitchContact::InitNewTarget( const KDL::Frame& Rt1, 
				   const KDL::Frame& Rt2 ){
  
  tf::Transform tfRt;
  tf::transformKDLToTF( Rt1, tfRt );
  
  geometry_msgs::Twist msgvw;   
  msgvw.linear.x = 0.0; 
  msgvw.linear.y = 0.0;
  msgvw.linear.z = 0.0; 
  msgvw.angular.x = 0.0;
  msgvw.angular.y = 0.0;
  msgvw.angular.z = 0.0;
  
  geometry_msgs::Twist msgvw_max;
  double max_linear_vel = 0.025;    // was 0.01 
  double max_angular_vel = 0.01;
  msgvw_max.linear.x = max_linear_vel; 
  msgvw_max.linear.y = max_linear_vel;
  msgvw_max.linear.z = max_linear_vel; 
  msgvw_max.angular.x = max_angular_vel;
  msgvw_max.angular.y = max_angular_vel; 
  msgvw_max.angular.z = max_angular_vel;
  
  trajectory.InitializeMode( tfRt, msgvw, msgvw_max );
  NewTarget( Rt2 );
  
}
*/

//! Pure virtual Approach() implementation
void StitchContact::Approach() {
  double approach_z = -0.01;
  KDL::Frame tmp = Rts_offset * KDL::Frame( KDL::Vector( 0.0, 0.0, approach_z ) );
  NewTarget(  tmp);
  std::cout << "new approach target:" << tmp.p[0] << " "<< tmp.p[1] << " " << tmp.p[2]<<std::endl;
}

//! Pure virtual Hover() implementation
void StitchContact::Hover() {
  double hover_z = -0.005;
  NewTarget( Rts_offset * KDL::Frame( KDL::Vector( 0.0, 0.0, hover_z ) ) );
}

//! Main Evaluate()
//! This implementation includes the accept_stitch and HOVER states
StitchBase::State StitchContact::Evaluate( KDL::Frame& Rt,
					   KDL::Twist& vw,
					   KDL::JntArray& q,
					   KDL::JntArray& qd,
					   const KDL::Vector& f,
					   bool accept_stitch,
					   bool repeat_stitch,
					   const geometry_msgs::Vector3& offset_vector,
					   const geometry_msgs::Vector3& contact_vector, 
					   const geometry_msgs::Vector3& contact_force, 
					   const geometry_msgs::Vector3& tension_vector, 
					   const geometry_msgs::Vector3& tension_force,
					   bool accept_tension
					   ) {
  
  StitchBase::State ret_state = IDLE;
  KDL::Vector offset( offset_vector.x, offset_vector.y, offset_vector.z);
  Rts_offset = Rts * KDL::Frame( offset ); //offset in tool frame
  
  contact_d = 0.006;//contact_vector.z; 
  contact_f = 1.5;    //contact_force.z;
  tension_d = 0.12;  //tension_vector.z;
  tension_f = 10;    //tension_force.z;
  
  switch( state ) {
    
  case START: {
	// this only initiates a trajectory to reach above the tissue
    ret_state = START;
    Rts_offset.M = Rts.M; // This should solve rotation to contact problem
    StitchContact::Start( Rt );
    state = APPROACH;    // Next state:
    std::cout << name << " Approach ..." << std::endl;
    break;
  }
    
  case APPROACH: {
    ret_state = APPROACH;
    if( EvaluateTrajectory( Rt ) == Trajectory::EXPIRED ) {
      state = HOVER;     // Next state
      std::cout << name << " Hover ..." << std::endl;
      StitchContact::Hover();
    }
	// IK not doing anything now since it is empty
    InverseKinematics( Rt, q );
    break;
  }
    
  case HOVER: {
    ret_state = HOVER;        
    
    if( EvaluateTrajectory( Rt ) == Trajectory::EXPIRED ) {
      if( accept_stitch ){// this quickly passes since I set the accept stitch equal to true
	Rts_offset.M = Rts.M; //
	state = CONTACT;
	std::cout << name << " Contact..." << std::endl;
	NewTarget(Rts_offset*KDL::Frame(KDL::Vector(0.0, 0.0,contact_d)));
	f_ref = f;
      }
      else {
	state = HOVER;
	StitchContact::Hover();
      }
    }
    InverseKinematics( Rt, q );
    break;
  }
    
  case CONTACT: {
    ret_state = CONTACT;
    KDL::Vector df = f - f_ref;
    if( EvaluateTrajectory( Rt ) == Trajectory::EXPIRED || df.Norm() > contact_f ) {
      //std::cout << df << " " contact_f << std::endl;
      if( df.Norm() > contact_f ) {
	ROS_INFO_STREAM( "Measured contact force = " << df.Norm() <<
			 " > " << contact_f );
	NewTarget( Rt );
      }
      else {
	ROS_INFO_STREAM( "Contact force " << df.Norm() <<
			 " was not larger than set threshold " << contact_f );
      }
      
      if ( repeat_stitch == 1 ) {
	state = HOVER;
	std::cout << name << " Hover ..." << std::endl;
	StitchContact::Hover();
      }
      else {
	if ( accept_stitch == 1 ) {
	  state = STITCH_FIRE;
	  std::cout << name << " StitchFire ..." << std::endl;
	}
	else { state = CONTACT; }
      }
    }
    InverseKinematics( Rt, q );
    break;
  }
    
  case STITCH_FIRE: {
    ret_state = STITCH_FIRE;
    if ( EvaluateTrajectory( Rt ) == Trajectory::EXPIRED ) {
      if ( repeat_stitch == 1 ) {
	state = HOVER; 
	std::cout << name << " Hover without stitch ..." << std::endl;
	StitchContact::Hover();
          }
      if ( accept_stitch == 1 ) {
	state = STITCH; // Next state
	std::cout << name << " Stitch ... " << std::endl;
	stitch_start_time = ros::Time::now();
      }
      else {
	state = STITCH_FIRE;
      }
    }
    InverseKinematics( Rt, q);
    break;
  }
    
  case STITCH: {
    ret_state = STITCH;
    // TODO: if( EvaluateTrajectory( Rt ) == Trajectory::EXPIRED )? /AS
    EvaluateTrajectory( Rt );
    
    if ( repeat_stitch == 1 ) {
      state = HOVER;
      std::cout << name << " Hover without stitch ..." << std::endl;
      StitchContact::Hover();
    }
    //TODO: else
    
    if( stitch_duration < ros::Time::now() - stitch_start_time ) {
      state = LIFT; // Next state
      std::cout << name << " Lift..." << std::endl;
      NewTarget( Rt * KDL::Frame( KDL::Vector(0.0, 0.0, -0.001) ) );
    }
    InverseKinematics( Rt, q );
    break;
  }
    
  case LIFT: {
    ret_state = LIFT;
    if ( repeat_stitch == 1 ) {
      state = HOVER;
      std::cout << name << " Repeating before LIFT ended ..." << std::endl;
      StitchContact::Hover();
    }
    else {
      if( EvaluateTrajectory( Rt ) == Trajectory::EXPIRED ) {
	state = TENSION; // Next state
	std::cout << name << " Tension..." << std::endl;
	KDL::Frame Rt_t( Rts.M, Rt.p );
	NewTarget( Rt * KDL::Frame( KDL::Vector( 0.0, 0.0, -tension_d ) ) );
	f_ref = f;
      }
    }
    InverseKinematics( Rt, q );
    break;
  }
    
  case TENSION: {
    ret_state = TENSION;
    KDL::Vector df = f - f_ref;
    
    if ( repeat_stitch == 1 ) {
      state = HOVER;
      std::cout << name << " Repeating before TENSION ended ..." 
		<< std::endl;
      StitchContact::Hover();
    }
    else {
      if( ( EvaluateTrajectory( Rt ) == Trajectory::EXPIRED ) || 
	  ( tension_f < df.z() ) || ( accept_tension == true )  ) {
	if( tension_f < df.z() ) {
	  ROS_INFO_STREAM( "Measured TENSION force = " << df.z() << " > "
			   << tension_f );
	  NewTarget( Rt );
	}
	state = RELEASE; // Next state
	std::cout << name << " Release..." << std::endl;
	//KDL::Frame Rt_t( Rts.M, Rt.p );
	NewTarget( Rt * KDL::Frame( KDL::Vector( 0.0, 0.0, RELEASE_MM ) ) );
      }
    }
    InverseKinematics( Rt, q );
    break;
  }
    
  case RELEASE: {
    ret_state = RELEASE;
    if ( repeat_stitch == 1 ) {
      state = HOVER;
      std::cout << name << " Repeating before RELEASE ended ..."
		<< std::endl;
      StitchContact::Hover();
    }
    else {
      if( EvaluateTrajectory( Rt ) == Trajectory::EXPIRED ) {
	state = FINISHED; // Next state
	std::cout << name << " Finished..." << std::endl;
      }
    }
    InverseKinematics( Rt, q );
    break;
  }
    
  case FINISHED: {
    ret_state = FINISHED;
    EvaluateTrajectory( Rt );
    if ( repeat_stitch == 1 ) {
      state = HOVER;
      std::cout << name << " Repeating before FINISHED ..." << std::endl;
      StitchContact::Hover();
    }
    else {
    }
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

