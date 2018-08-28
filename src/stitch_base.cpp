
#include <icra19_suture_rtt/stitch_base.hpp>
#include <tf_conversions/tf_kdl.h>

void StitchBase::print( const KDL::Frame& Rt, const std::string& s ){
  std::ios::fmtflags flags = std::cout.setf( std::ios::fixed );
  std::streamsize precision = std::cout.precision(5);
  std::cout << s;
  for( size_t i=0; i<3; i++ ){
    std::cout << std::setw( 13 ) << Rt.M(i,0)
	      << std::setw( 13 ) << Rt.M(i,1)
	      << std::setw( 13 ) << Rt.M(i,2)
	      << std::setw( 13 ) << Rt.p(i)
	      << std::endl;
  }
  std::cout.precision( precision );
  std::cout.setf( flags );
}
// TODO: Move these to stitch_helper.cpp /AS
double jointlimit[] = {
  170.0*M_PI/180,
  120.0*M_PI/180,
  170.0*M_PI/180,
  120.0*M_PI/180,
  170.0*M_PI/180,
  120.0*M_PI/180,
  170.0*M_PI/180
};

std::ostream& operator<<( std::ostream& os, const StitchBase::State& state ) {
  switch( state ){
  case StitchBase::IDLE:       os << "IDLE";       break;
  case StitchBase::START:      os << "START";      break;
  case StitchBase::APPROACH:   os << "APPROACH";   break;
  case StitchBase::CONTACT:    os << "CONTACT";    break;
  case StitchBase::SLIDE:      os << "SLIDE";      break;
  case StitchBase::JOINTSTATE: os << "JOINTSTATE"; break;
  case StitchBase::STITCH:     os << "STITCH";     break;
  case StitchBase::LIFT:       os << "LIFT";       break;
  case StitchBase::TENSION:    os << "TENSION";    break;
  case StitchBase::RELEASE:    os << "RELEASE";    break;
  case StitchBase::FINISHED:   os << "FINISHED";   break;
  case StitchBase::HOVER:      os << "HOVER";      break;
  case StitchBase::STITCH_FIRE:os << "STITCH_FIRE";break;
  case StitchBase::SLIDE_ROT:  os << "SLIDE_ROT";  break;                                   
  }
  return os;
}

geometry_msgs::Vector3 zero3() {
  geometry_msgs::Vector3 result;
  result.x = 0.0;
  result.y = 0.0;
  result.z = 0.0;
  return result;
}

StitchBase::StitchBase( const KDL::Frame& Rts, double f, double d, double t ):
  state( START ),
  trajectory( 0.002 ),
  Rts( Rts ),
  tension_f( f ), 
  tension_d( d ), 
  stitch_duration( t ) {
  
  //chain = CNMC::LWREndo360();
  //fk = new KDL::ChainFkSolverPos_recursive( chain );
  //ik = new ChainIkSolverPos_RCM_JL( chain, KDL::Vector::Zero() );
  
  }

StitchBase::~StitchBase() { }

void StitchBase::ForwardKinematics( const KDL::JntArray& q, KDL::Frame& Rt ) { 
  
  //fk->JntToCart( q, Rt ); 
  
}

void StitchBase::InverseKinematics( const KDL::Frame& Rt, KDL::JntArray& qold ) {
  /*
  KDL::JntArray qnew = qold;
  if( 0 <= ik->CartToJnta( qold, Rt, qnew ) ) {       
    qold = qnew; 
  }
  else { 
    std::cout << "ERROR" << std::endl; 
  }
  */
}

void StitchBase::NewTarget( const KDL::Frame& kdlRt ) {
  tf::Transform tfRt;
  tf::transformKDLToTF( kdlRt, tfRt );
  geometry_msgs::Twist vw;
  vw.linear.x = vw.linear.y = vw.linear.z = 0.0;
  vw.angular.x = vw.angular.y = vw.angular.z = 0.0;
  trajectory.Push( tfRt, vw );
}

void StitchBase::NewTarget( const KDL::JntArray& q ){
  
  sensor_msgs::JointState js;
  js.name.resize( q.rows() );
  js.position.resize( q.rows() );
  js.velocity.resize( q.rows() );
  js.effort.resize( q.rows() );
  
#if 0 
  // DEBUG
  std::cout << "q = [ ";
  for( size_t i=0; i<q.rows(); i++)
    std::cout << q(i)*180/M_PI << "  ";
  std::cout << " ] (deg)" << std::endl;
#endif
  // Taking joint limit into account
  for( size_t i=0; i<q.rows(); i++ ) {
    if ( i<7 ) {
      if ( q(i)>jointlimit[i]  ) {
	js.position[i] = jointlimit[i];
	std::cout << "q("<<i<<") hits UPPER limit!" << std::endl;
      }
      else
	if ( q(i)<=-jointlimit[i] ) {
	  js.position[i] = -jointlimit[i];
	  std::cout << "q("<<i<<") hits LOWER limit!" << std::endl;
	}
	else {
	  js.position[i] = q(i);
	}
    }
    else { // Endo360 articulation check done in UTA
      js.position[i] = q(i);
    }
    
    js.velocity[i] = 0.0;
    js.effort[i] = 0.0;
    js.name[i] = std::string( "J" );
  }
  
  trajectory.Push( js );
  
}


Trajectory::Errno StitchBase::EvaluateTrajectory( KDL::Frame& kdlRt ) {
  
  tf::Transform tfRt;
  KDL::Twist vw;
  Trajectory::Errno err;
  err = trajectory.Evaluate( tfRt,vw );
  tf::transformTFToKDL( tfRt, kdlRt );

  return err;
  
}

Trajectory::Errno StitchBase::EvaluateTrajectory( KDL::JntArray& kdlq ){
  
  sensor_msgs::JointState js;
  Trajectory::Errno err;
  err = trajectory.Evaluate( js );
  kdlq.resize( js.position.size() );
  
  for( size_t i=0; i<js.position.size(); i++ ) { 
    kdlq(i) = js.position[i]; 
  }
  
  return err;
  
}



