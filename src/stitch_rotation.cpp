#include <icra19_suture_rtt/stitch_rotation.hpp>
#include <tf_conversions/tf_kdl.h>

StitchRotation::StitchRotation( const KDL::Frame& Rts,      // desired frame
				const KDL::JntArray& _dqs,
				double cf,
				double cd,
				double tf,
				double td ):
  StitchContact( Rts, cf, cd, tf, td ),
  dqs( _dqs ) { 
  name = std::string("StitchRotation"); 
  }

StitchRotation::~StitchRotation() {}

  StitchBase::State StitchRotation::Evaluate( KDL::Frame& Rt,
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
    Rts_offset = Rts * KDL::Frame( offset );
    
    contact_d = contact_vector.z;
    contact_f = contact_force.z;
    tension_d = tension_vector.z;
    tension_f = tension_force.z;
    
    switch( state ) {
      
    case START: {
      ret_state = START;
      
      state = JOINTSTATE;
      std::cout << name << " Joint state..." << std::endl;
      
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
      
      //std::vector<double> qdmax( q.rows(), 0.2 );
      std::vector<double> qdmax( q.rows(), 0.5 );
      std::vector<double> qddmax( q.rows(), 10.0 );
      std::vector<double> qdddmax( q.rows(), 10.0 );
      trajectory.InitializeMode( js, qdmax, qddmax, qdddmax );
      
      KDL::JntArray qs( q );
      for( size_t i=0; i<qs.rows(); i++ )
        {
          qs(i) += dqs(i);
        }
      NewTarget( qs );
      
      InverseKinematics( Rt, qs ); 
      break;
    }
      
    case JOINTSTATE: {
      ret_state = JOINTSTATE;
      if( EvaluateTrajectory( q ) == Trajectory::EXPIRED ){
	state = FINISHED;
	std::cout << name << " Finished..." << std::endl;
        
	ForwardKinematics( q, Rt);
        
	tf::Transform tfRt;
	tf::transformKDLToTF( Rt, tfRt );
	geometry_msgs::Twist msgvw;
	msgvw.linear.x = vw.vel.x(); msgvw.linear.y = vw.vel.y();
	msgvw.linear.z = vw.vel.z(); msgvw.angular.x = vw.rot.x();
	msgvw.angular.y = vw.rot.y();  msgvw.angular.z = vw.rot.z();
	
	geometry_msgs::Twist msgvw_max;
	msgvw_max.linear.x = 0.01; msgvw_max.linear.y = 0.01;
	msgvw_max.linear.z = 0.01; msgvw_max.angular.x = 0.01;
	msgvw_max.angular.y = 0.01; msgvw_max.angular.z = 0.01;
	
	trajectory.InitializeMode( tfRt, msgvw, msgvw_max );
	NewTarget( Rt );
      }
      ForwardKinematics( q, Rt );
      
      break; 
    }
    case FINISHED: {
      ret_state = FINISHED;
      if ( repeat_stitch == 1 ) {
	state = START;
	std::cout << name << " Repeating before FINISHED ..." << std::endl;
	StitchRotation::Hover();
      }
      
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
  return ret_state;
}



