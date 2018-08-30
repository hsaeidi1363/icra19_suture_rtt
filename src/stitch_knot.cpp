#include <icra19_suture_rtt/stitch_knot.hpp>
#include <tf_conversions/tf_kdl.h>

StitchKnot::StitchKnot( const KDL::Frame& Rts,      // desired frame
			size_t nloops,
			double cf,
			double cd,
			double tf,
			double td ):
  StitchContact( Rts, cf, cd, tf, td ),
  num_loops( nloops+1 ),
  cur_loop( 0 ),
  SURGEONS_KNOT( true ) {
  name = std::string("StitchKnot"); 
}

StitchKnot::~StitchKnot() {}

//! Specialized Hover for Knot
void StitchKnot::KnotHover() {
  double hover_z = -0.02;
  NewTarget( Rts_offset * KDL::Frame( KDL::Vector( 0.0, 0.0, hover_z ) ) );
}

StitchBase::State StitchKnot::Evaluate( KDL::Frame& Rt, 
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
  
  double dq6;
  StitchBase::State ret_state = IDLE;
  //std::cout << "inside the evaluate function "<<std::endl;
  KDL::Vector offset( offset_vector.x, offset_vector.y, offset_vector.z);
  Rts_offset = Rts * KDL::Frame( offset );
  
  contact_d = contact_vector.z;
  contact_f = contact_force.z;
  tension_d = tension_vector.z;
  tension_f = tension_force.z;
  
  double ROTATION_ANGLE = 0.5; // was M_PI_2/2
  
  if (cur_loop == 1 ) {
    dq6 = ROTATION_ANGLE;
    //std::cout << "current loop: 1 "<<std::endl;
  }
  else if (cur_loop < num_loops) {
    dq6 = 0;
    //std::cout << "current loop: less than num_loops "<<std::endl;
  }
  else {
    dq6 = -ROTATION_ANGLE;
    //std::cout << "current loop: negative rotation "<<std::endl;
  }
  
  if( cur_loop <= num_loops ) {
    
    switch( state ) {
      
    case START: {
      ret_state = START;
      //overwrite orientation of commanded Rts
      Rts.M = Rt.M;
      
      std::cout << name << " Joint state..." << std::endl;
      state = JOINTSTATE;
      std::cout << "number of rows in q: " << q.rows() <<std::endl;
      sensor_msgs::JointState js;
      js.name.resize( q.rows() );
      js.position.resize( q.rows() );
      js.velocity.resize( q.rows() );
      js.effort.resize( q.rows() );
      
      // With joint limit /AS 10-29-2014
      for( size_t i=0; i<q.rows(); i++ ){
	js.position[i] = q(i);
	js.velocity[i] = qd(i);
	js.effort[i] = 0.0;
	js.name[i] = std::string( "J" );
      }
      
      std::vector<double> qdmax( q.rows(), 0.5 ); // was 0.2
      std::vector<double> qddmax( q.rows(), 10.0 );
      std::vector<double> qdddmax( q.rows(), 10.0 );
      trajectory.InitializeMode( js, qdmax, qddmax, qdddmax );
      std::cout << "initialized the joint trajectory generator"<< std::endl;
      KDL::JntArray qs( q );
      
      if ( ( dq6 + qs(6) ) > (170.0*M_PI/180) ) {
	ROS_INFO_STREAM( "START: Will hit joint limit!" );
	dq6 = (170.0*M_PI/180) - qs( 6 );
      }
      
      ROS_INFO_STREAM( "dq6 = " << dq6);
      qs(6) += dq6;
      //TODO: Keep original qs(6) in a private var to avoid joint limit problems
      
      NewTarget( qs );
      InverseKinematics( Rt, qs ); 
      break;
    }
      
    case JOINTSTATE: {
      ret_state = JOINTSTATE;
      
      if( EvaluateTrajectory( q ) == Trajectory::EXPIRED ) {
	
	ForwardKinematics( q, Rt);
	
	tf::Transform tfRt;
	tf::transformKDLToTF( Rt, tfRt );
	geometry_msgs::Twist msgvw;
	msgvw.linear.x = vw.vel.x(); msgvw.linear.y = vw.vel.y();
	msgvw.linear.z = vw.vel.z(); msgvw.angular.x = vw.rot.x();
	msgvw.angular.y = vw.rot.y();  msgvw.angular.z = vw.rot.z();
	
	geometry_msgs::Twist msgvw_max;
	msgvw_max.linear.x = 0.01; 
	msgvw_max.linear.y = 0.01;
	msgvw_max.linear.z = 0.02; // was 0.01
	msgvw_max.angular.x = 0.01; 
	msgvw_max.angular.y = 0.01; 
	msgvw_max.angular.z = 0.01;
	
	
	state = APPROACH;
	std::cout << name << " APPROACH ... " << std::endl;
	trajectory.InitializeMode( tfRt, msgvw, msgvw_max );
	NewTarget( Rt );
	
      }
      ForwardKinematics( q, Rt );
      break;
    }
      
    case APPROACH: {
      ret_state = APPROACH;
      if( EvaluateTrajectory( Rt ) == Trajectory::EXPIRED ){
	if (cur_loop == num_loops) {
	  state = RELEASE ;
	  std::cout << name << " Release ..." << std::endl;
	  NewTarget( Rt * KDL::Frame( KDL::Vector( 0.0, 0.0, RELEASE_MM ) ) );
	}
	else {
	  Rts_offset.M = Rt.M; // TODO: Did this solve cur_loop > 0?
	  if ( cur_loop == 0 ) {
	    state = CONTACT;
	    std::cout << name << " First Contact ..." << std::endl;
	    NewTarget( Rts_offset * KDL::Frame( KDL::Vector(0.0, 0.0, contact_d) ) );
	    f_ref = f;
	  }
	  else {
	    ROS_INFO_STREAM( " cur_loop = " << cur_loop );
	    state = HOVER;
	    std::cout << name << " HOVER ... " << std::endl;
	    StitchKnot::KnotHover();
	    //StitchKnot::Hover();
	  }
	}
      }
      InverseKinematics( Rt, q );
      break; 
    }
      
    case HOVER: {
      ret_state = HOVER;
      if( EvaluateTrajectory( Rt ) == Trajectory::EXPIRED ) {
	
	if ( cur_loop == 0 ) {
	  if ( accept_stitch == 1 ) {
	    state = CONTACT;
	    std::cout << name << " Contact..." << std::endl;
	    Rts_offset.M = Rt.M;
	    NewTarget( Rts_offset * KDL::Frame( KDL::Vector(0.0, 0.0, contact_d) ) );
	    f_ref = f;
	  }
	  else {
	    state = HOVER;
	    StitchKnot::KnotHover();
	    //StitchKnot::Hover();
	  }
	}
	else { // cur_loop != 0
	  if ( accept_stitch == 1 ) {
	    state = STITCH_FIRE;
	    std::cout << name << " StitchFire ... " << std::endl;
	  }
	}
      }
      InverseKinematics( Rt, q );
      break;
    }
      
    case CONTACT: {
      ret_state = CONTACT;
      KDL::Vector df = f - f_ref;
      if( EvaluateTrajectory( Rt ) == Trajectory::EXPIRED || df.z() < contact_f ){
	if( df.z() < contact_f ) {
	  ROS_INFO_STREAM( "Measured contact force = " << df.z() <<
			   " < " << contact_f );
	  NewTarget( Rt );
	}
	else {
	  ROS_INFO_STREAM( "Contact force " << df.z() <<
			   " was not larger than set threshold " << contact_f );
	}
	if ( repeat_stitch == 1 ) {
	  cur_loop = 0; // TODO: Check
	  state = HOVER;
	  std::cout << name << " Hover ..." << std::endl;
	  StitchKnot::KnotHover();
	  //StitchKnot::Hover();
	}
	else {
	  if ( accept_stitch == 1 ) {
	    state = STITCH_FIRE;
	    std::cout << name << " StitchFire ..." << std::endl;
	  }
	  else {
	    state = CONTACT;
	  }
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
	  StitchKnot::KnotHover();
	  //StitchContact::Hover();
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
      EvaluateTrajectory( Rt );
      
      if ( repeat_stitch == 1 ) {
	state = HOVER;
	std::cout << name << " Hover without stitch ..." << std::endl;
	StitchKnot::KnotHover();
	//StitchContact::Hover();
      }
      //TODO: else, here?
      
      if( stitch_duration < (ros::Time::now() - stitch_start_time) ) {
	// Done with first throw
	// Preparing for second throw HERE
	if ( (cur_loop == 1) && (SURGEONS_KNOT == true) ) {
	  stitch_start_time = ros::Time::now();
	  state = STITCH;
	  SURGEONS_KNOT = false;
	}
	else {
	  // Done with second throw
	  std::cout << name << " Lift..." << std::endl;
	  NewTarget( Rt * KDL::Frame( KDL::Vector( 0.0, 0.0, -RELEASE_MM ) ) );
	  state = LIFT;
	}
      }
      InverseKinematics( Rt, q );
      break; 
    }
      
    case LIFT: {
      ret_state = LIFT;
      if ( repeat_stitch == 1 ) {
	state = HOVER;
	std::cout << name << " Repeating before LIFT ended ..." << std::endl;
	StitchKnot::KnotHover();
	//StitchContact::Hover();
      }
      else {
	if( EvaluateTrajectory( Rt ) == Trajectory::EXPIRED ){
	  state = TENSION;
	  std::cout << name << " Tension..." << std::endl;
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
	std::cout << name << " Repeating before TENSION ended ..." << std::endl;
	StitchKnot::KnotHover();
	//StitchContact::Hover();
      }
      else {
	if( ( EvaluateTrajectory( Rt ) == Trajectory::EXPIRED ) || ( tension_f < df.z() ) || ( accept_tension == true )  ) {
	  if( tension_f < df.z() ) {
	    ROS_INFO_STREAM( "Measured TENSION force = " << df.z() << " > " << tension_f );
	    NewTarget( Rt );
	  }
	  if (cur_loop == num_loops) {
	    state = RELEASE;
	    std::cout << name << " Release..." << std::endl;
	    NewTarget( Rt * KDL::Frame( KDL::Vector( 0.0, 0.0, RELEASE_MM ) ) );
	  }
	  else {
	    if( cur_loop < num_loops ){
	      cur_loop++;
	      state = START;
	      std::cout << name << " starting again ... loop " << (cur_loop-1) << "/" << (num_loops-1) << std::endl;
	    }
	  }
	}
      }
      InverseKinematics( Rt, q );
      break; 
    }
      
    case RELEASE: {
      ret_state = RELEASE;
      if( EvaluateTrajectory( Rt ) == Trajectory::EXPIRED ){
	state = FINISHED;
	std::cout << name << " Finished..." << std::endl;
      }
      InverseKinematics( Rt, q );
      break; 
    }
      
    case FINISHED: {
      ret_state = FINISHED;
      if (EvaluateTrajectory( Rt ) == Trajectory::EXPIRED){
	InverseKinematics( Rt, q );
      }
      break;
    }
    default: {
      ROS_INFO( "Default behavior. Switch case shouldn't end up here!" );
      break;
    }
    }
  }
  return ret_state;

}

