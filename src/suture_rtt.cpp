// Copyright  (C)  2013 Simon Leonard

// Author: Simon Leonard and Hamed Saeidi
// Maintainer: Simon Leonard and Hamed Saeidi

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include <icra19_suture_rtt/suture_rtt.hpp>

#include <rtt_rosparam/rosparam.h>
#include <urdf/model.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <star_rtt/ChainIkSolverPos_RCM.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl_conversions/kdl_msg.h>
#include <tf_conversions/tf_kdl.h>
#include <eigen_conversions/eigen_kdl.h>

#include<icra19_suture_rtt/stitch_contact.hpp>
#include<icra19_suture_rtt/stitch_knot.hpp>
#include<icra19_suture_rtt/stitch_base.hpp>

#include <pcl/common/transforms.h>


#include <rtt/Component.hpp>         // use for ORO_CREATE_COMPONENT

void read_line( std::ifstream& ifs, double xyzrpy[6] ){
  ifs >> xyzrpy[0] >> xyzrpy[1] >> xyzrpy[2];// >> xyzrpy[3] >> xyzrpy[4] >> xyzrpy[5];
  std::cout << "point: " << xyzrpy[0] << " " << xyzrpy[1] << " " << xyzrpy[2] << std::endl;
  xyzrpy[0] = xyzrpy[0] + 0.005538136699281;
  xyzrpy[1] = xyzrpy[1] + 0.002872239501684;
  xyzrpy[2] = xyzrpy[2] - 0.002225613451732 - 0.003;
}


suture_rtt::suture_rtt( const std::string& name ):

  TaskContext( name, PreOperational ),

  ts( NULL ),
  ticks( 0 ),

  port_iiwa_msr_joint_pos( "IIWA joint positions (telemetry)" ),
  port_iiwa_msr_joint_vel( "IIWA joint velocities (telemetry)" ),
  port_iiwa_msr_joint_trq( "IIWA joint torques (telemetry)" ),
  port_iiwa_msr_cart_pos( "IIWA Cartesian position (telemetry)" ),
  port_iiwa_msr_cart_ft( "IIWA Cartesian wrench (telemetry)" ),
  port_iiwa_cmd_joint_pos( "IIWA joint positions (commands)" ),
  port_iiwa_cmd_joint_vel( "IIWA joint velocities (commands)" ),
  port_iiwa_cmd_joint_pos_vel( "IIWA joint positions velocities (commands)" ),
  port_iiwa_cmd_cart_pos( "IIWA Cartesian position (commands)" ),

  port_msr_jointstate( "STAR joint state (telemetry)" ),
  port_cmd_jointstate( "STAR joint state (command)" ),

  port_tool_msr_jointstate( "Tool joint state (telemetry)" ),
  port_tool_msr_cart_ft( "Tool Cartesian wrenchy (telemetry)" ),
  port_tool_cmd_jointstate( "Tool joint state (command)" ),

  // 70 degrees for 1650000
  enc_to_rad( 1.2217 / 165000 ),
  rad_to_enc( 165000 / 1.2217 ),

  init_rcm( false ),
  test_traj_initialized(false),
  test_stitch_defined(false),
  go(false),
  test_stitch_seq(1),


  trajectory( 0.01 ),
  play_file( false ){
  
  ts = RTT::os::TimeService::Instance();
  
  addPort( "MsrIIWAJointPos",   port_iiwa_msr_joint_pos );
  addPort( "MsrIIWAJointVel",   port_iiwa_msr_joint_vel );
  addPort( "MsrIIWAJointTrq",   port_iiwa_msr_joint_trq );
  addPort( "MsrIIWACartPos",    port_iiwa_msr_cart_pos );
  addPort( "MsrIIWAWrench",     port_iiwa_msr_cart_ft );
  
  addPort( "CmdIIWAJointPos",   port_iiwa_cmd_joint_pos );
  addPort( "CmdIIWAJointVel",   port_iiwa_cmd_joint_vel );
  addPort( "CmdIIWAJointPosVel",port_iiwa_cmd_joint_pos_vel );
  addPort( "CmdIIWACartPos",    port_iiwa_cmd_cart_pos );
  
  addPort( "MsrToolJointState", port_tool_msr_jointstate );
  addPort( "MsrToolWrench",     port_tool_msr_cart_ft );
  addPort( "CmdToolJointState", port_tool_cmd_jointstate );
  
  addPort( "MsrSTARJointState", port_msr_jointstate );
  addPort( "CmdSTASJointState", port_cmd_jointstate );
  
  // for debugging the plan points on the tissue
  addPort( "Plan",    port_plan );
  addPort( "Markers", port_markers );


  addProperty( "robot_description_name", robot_description_name );
  addProperty( "robot_description", robot_description );
  addProperty( "ref_frame", ref_frame );
  addProperty( "tcp_frame", tcp_frame );
  
  
  addOperation( "GetJointsPos", &suture_rtt::getJointPos, this, RTT::OwnThread );
  addOperation( "GetJointsVel", &suture_rtt::getJointVel, this, RTT::OwnThread );
  addOperation( "SetJointsPos", &suture_rtt::setJointPos, this, RTT::OwnThread );
  addOperation( "SetJointsVel", &suture_rtt::setJointVel, this, RTT::OwnThread );
  addOperation( "SetJointsPowVel", &suture_rtt::setJointPosVel, this, RTT::OwnThread );

  addOperation( "GetCartRot",   &suture_rtt::getCartRot, this, RTT::OwnThread );
  addOperation( "GetCartPos",   &suture_rtt::getCartPos, this, RTT::OwnThread );
  addOperation( "GetCartFrame", &suture_rtt::getCartFrame, this, RTT::OwnThread );

  addOperation( "SetCartXYZ",   &suture_rtt::setCartXYZ, this, RTT::OwnThread );
  addOperation( "SetCartFrame", &suture_rtt::setCartFrame, this, RTT::OwnThread );

  addOperation( "SetRCM", &suture_rtt::setRCM, this, RTT::OwnThread );

  addOperation( "PlayFile", &suture_rtt::playFile, this, RTT::OwnThread );

}

suture_rtt::~suture_rtt(){
  RTT::os::TimeService::Release();
}

bool suture_rtt::configureHook() { 

  // look for the tf component
  RTT::TaskContext* peer = this->getPeer("tf");
  if( peer == NULL ){
    RTT::log(RTT::Warning) << "Component has no peer tf " << RTT::endlog();
    return false;
  }

  // look for the lookup transform service (query /tf)
  if( !peer->operations()->hasMember( "lookupTransform" ) ){
    RTT::log(RTT::Warning) << "Peer tf has no operation lookupTransform " 
      << RTT::endlog();
    return false;
  }
  else
  { lookup_tf = peer->provides()->getOperation( "lookupTransform" ); }


  // Get the parameter server
  boost::shared_ptr<rtt_rosparam::ROSParam> rosparam = 
    this->getProvider<rtt_rosparam::ROSParam>( "rosparam" );

  std::cout << "robot description: " << robot_description << std::endl; 

  if( rosparam ){

    if( rosparam->getParam( robot_description, "robot_description_name" ) ){ 

      urdf::Model urdf_model;
      if( urdf_model.initString( robot_description_name ) ){
	urdf_name = urdf_model.getName();

        KDL::Tree kdl_tree;
        if( kdl_parser::treeFromUrdfModel( urdf_model, kdl_tree ) ){

          if( kdl_tree.getChain( ref_frame, tcp_frame, chain ) ){

            RTT::log(RTT::Info) << "Found kinematics chain" << RTT::endlog();

            //kinematics solvers
            std::cout<<"constructing kinematics solvers"<<std::endl;
            fk_solv = new KDL::ChainFkSolverPos_recursive( chain );
            vw_solv = new KDL::ChainIkSolverVel_pinv( chain );
            ik_solv = new KDL::ChainIkSolverPos_NR( chain, *fk_solv, *vw_solv );
            msr_joint_pos.resize( chain.getNrOfJoints() );
            msr_joint_vel.resize( chain.getNrOfJoints() );
            msr_joint_trq.resize( chain.getNrOfJoints() );

          }
          else{ 
            RTT::log(RTT::Warning) << "No KDL chain" << RTT::endlog();
          }
        }
        else{
          RTT::log(RTT::Warning) << "KDL conversion fadailed" << RTT::endlog();
        }
      }
      else{
        RTT::log(RTT::Warning) << "Failed to parse URDF file" << RTT::endlog();
      }
    }
    else{ 
      RTT::log(RTT::Warning) << "robot description not found" << RTT::endlog();
    }
  }
  else{
    RTT::log(RTT::Warning) << "rosparam not found" << RTT::endlog();
  }

  return true; 

}

bool suture_rtt::startHook() { return true; }

void suture_rtt::updateHook(){
  
  if( readIIWA() == suture_rtt::ESUCCESS && 
      readTool() == suture_rtt::ESUCCESS ){
  //  fk_solv->JntToCart( getJointPos(), msr_cart_pos );
	if(!go){
		ReadMarkers();
	}else{

	// psuedo code: first check if the traj is detected make a plan via suture array (including where each stitch should go)=> execute the finite state machine code one by one
	
		if(!test_traj_initialized && !plan.empty()){
			// find the current position and add some dummy targets to it
			KDL::Frame kdlRt;
			KDL::JntArray jointpositions = getJointPos();
			bool joints_zero =true;
			for (int i = 0; i < 7; ++i)
				joints_zero = joints_zero && ( (fabs(jointpositions(i)) < 0.001) ||  (fabs(jointpositions(i)) > 3.1415) );
	    		if(!joints_zero){
				fk_solv->JntToCart( jointpositions, kdlRt );
				std::cout <<"initial pose: " << kdlRt.p[0] << " " <<kdlRt.p[1] << " "<< kdlRt.p[2] << std::endl;		
				if(!test_stitch_defined){
					switch(test_stitch_seq){
						case 1: 
							// quick test for the contact stitch
							{
							double contact_force = 1.0;
							double contact_distance = 0.1;
							double tension_force = 2.0;
							double tension_distance = 0.2;
							kdlRt.p.x(0.1);
							kdlRt.p.z(0.56);
							kdlRt.p.x(plan.points[0].x);
							kdlRt.p.y(plan.points[0].y);
							kdlRt.p.z(plan.points[0].z);		
							test_stitch = new StitchContact( 
										kdlRt,
										contact_force,
										contact_distance,
										tension_force,
										tension_distance );
							}
							break;
						case 2:

							// quick test for the knot stitch
							{
							size_t nloops = 2;
							double contact_force = 1.0;
							double contact_distance = 0.1;
							double tension_force = 2.0;
							double tension_distance = 0.2;
							kdlRt.p.x(0.1);
							kdlRt.p.y(-0.54) ;
							kdlRt.p.z(0.56);
							kdlRt.p.x(plan.points[1].x);
							kdlRt.p.y(plan.points[1].y);
							kdlRt.p.z(plan.points[1].z);
							test_stitch = new StitchKnot(   kdlRt,
											nloops,								
											contact_force,
											contact_distance,
											tension_force,
											tension_distance); 
							}
							break;
						case 3: 
							// quick test for the contact stitch
							{
							double contact_force = 1.0;
							double contact_distance = 0.1;
							double tension_force = 2.0;
							double tension_distance = 0.2;
							kdlRt.p.x(0.1);
							kdlRt.p.z(0.56);
							kdlRt.p.x(plan.points[2].x);
							kdlRt.p.y(plan.points[2].y);
							kdlRt.p.z(plan.points[2].z);		
							test_stitch = new StitchContact( 
										kdlRt,
										contact_force,
										contact_distance,
										tension_force,
										tension_distance );
							}
							break;
						case 4: 
							// quick test for the contact stitch
							{
							double contact_force = 1.0;
							double contact_distance = 0.1;
							double tension_force = 2.0;
							double tension_distance = 0.2;
							kdlRt.p.x(0.1);
							kdlRt.p.z(0.56);
							kdlRt.p.x(plan.points[3].x);
							kdlRt.p.y(plan.points[3].y);
							kdlRt.p.z(plan.points[3].z);		
							test_stitch = new StitchContact( 
										kdlRt,
										contact_force,
										contact_distance,
										tension_force,
										tension_distance );
							}
							break;
						default:
							std::cout << "---finished the final step---- : " <<std::endl;
							break;
					}
					std::cout << "set a new target at : "<< kdlRt.p[0] << " "<<kdlRt.p[1] <<" " <<kdlRt.p[2] <<std::endl;
					test_stitch_defined = true;
				} 
				test_traj_initialized = true;
			}

		}else{
			  
				KDL::Frame tmp_Rt;
				fk_solv->JntToCart( getJointPos(),tmp_Rt );
			 	KDL::Twist vw;
				KDL::JntArray q = getJointPos();
				KDL::JntArray qd = getJointVel(); // what should we do with this in the FRI driver?
				KDL::Vector f(0.0, 0.0, 0.0);
				geometry_msgs::Vector3 offset;
				offset.x = 0.0; offset.y = 0.0; offset.z = 0.0;
				StitchBase::State state = test_stitch->Evaluate( tmp_Rt, 
										vw, 
										q, 
										qd, 
										f,
										true,
										false,
										offset ); // some of the variables are not actually set and the default zero values are called

			
				KDL::JntArray q_old = getJointPos();
				KDL::JntArray q_new;
				if(state == StitchBase::JOINTSTATE){
					q_new = q;
				}else{
					ik_solv->CartToJnt( q_old, tmp_Rt, q_new  );
				}

				setJointPos( q_new );
				if (state == StitchBase::FINISHED && test_stitch_seq < 5){		
					test_traj_initialized = false;
					test_stitch_defined = false;
					test_stitch_seq ++;
				}

			
		}
	
	}
    sensor_msgs::PointCloud2 ros_plan;
    pcl::toROSMsg( plan, ros_plan );
    ros_plan.header.stamp = ros::Time::now();
    ros_plan.header.frame_id = "/camera_color_optical_frame";
    port_plan.write( ros_plan );


    if( init_rcm ){

      tf::Transform tfRt;
      KDL::Twist vw;
      switch( trajectory.Evaluate( tfRt, vw ) ){
        case Trajectory::SUCCESS:
          {
            KDL::Frame kdlRt;
            tf::transformTFToKDL( tfRt, kdlRt );

            KDL::JntArray q_old = getJointPos();
            KDL::JntArray q_new;

            if( ik_solv->CartToJnt( q_old, kdlRt, q_new  ) == 0 )
            { setJointPos( q_new); }
            else
            { RTT::log(RTT::Warning) << "Inverse kinematics failed"
				     << RTT::endlog(); }
          }
          break;
        case Trajectory::EXPIRED:
          {
            init_rcm = false;
          }
          break;
        default:
          break;
      }      
    }
    else{ evaluate(); }
    
    publishJointState();
	//std::cout << "readIIWA says: " <<readIIWA()<<" readTool says: "<<readTool() << std::endl;

  }

}

void suture_rtt::stopHook(){}
void suture_rtt::cleanupHook(){}


suture_rtt::Errno suture_rtt::evaluate(){
  tf::Transform tfRt;
  KDL::Twist vw;
  switch( trajectory.Evaluate( tfRt, vw ) ){
  case Trajectory::SUCCESS:
    {
      KDL::Frame kdlRt;
      tf::transformTFToKDL( tfRt, kdlRt );
      
      KDL::JntArray q_old = getJointPos();
      KDL::JntArray q_new, qd_new( chain.getNrOfJoints() );
      
      if( ik_solv->CartToJnt( q_old, kdlRt, q_new  ) == 0 ){
	KDL::ChainIkSolverVel_pinv vw_tmp(chain);
	vw_tmp.CartToJnt( q_new, vw, qd_new );
	setJointPosVel( q_new, qd_new );
	//setJointPos( q_new );
      }
      else
	{ RTT::log(RTT::Warning) << "Inverse kinematics failed"
				 << RTT::endlog(); }
    }
    break;
  case Trajectory::EXPIRED:
    {
    }
    break;
  default:
    break;
  }      
  return suture_rtt::ESUCCESS;
}

KDL::Frame suture_rtt::cancelRoll( const KDL::Frame& Rt0n ){

  KDL::Rotation Rn0 = Rt0n.M.Inverse();
  KDL::Vector d0 = Rt0n.p - rcm;
  KDL::Vector dn = Rn0 * d0;

  double dnz = dn.z();
  double dny = dn.y();
  double rx = atan2( dny, dnz );
  KDL::Frame Rt0nrcm( Rt0n );


  Rt0nrcm.M = Rt0nrcm.M * KDL::Rotation::RotX( -rx );

  return Rt0nrcm;

}

KDL::Frame suture_rtt::cancelPitch( const KDL::Frame& Rt0n ){

  KDL::Rotation Rn0 = Rt0n.M.Inverse();
  KDL::Vector d0 = Rt0n.p - rcm;
  KDL::Vector dn = Rn0 * d0;

  double dnz = dn.z();
  double dnx = dn.x();
  double ry = atan2( dnx, dnz );
  KDL::Frame Rt0nrcm( Rt0n );


  Rt0nrcm.M = Rt0nrcm.M * KDL::Rotation::RotY( ry );

  return Rt0nrcm;

}

std::vector<double> suture_rtt::getXYZ( const std::vector<double>& xyz ){

  KDL::Frame kdlRt;
  fk_solv->JntToCart( getJointPos(), kdlRt );

  KDL::Frame RCM = kdlRt * KDL::Frame( KDL::Vector(xyz[0], xyz[1], xyz[2]) );
  rcm = RCM.p;

  std::vector<double> std_rcm(3);
  std_rcm[0] = rcm(0);
  std_rcm[1] = rcm(1);
  std_rcm[2] = rcm(2);

  return std_rcm;
}

void suture_rtt::playFile( const std::string& filename ){

  std::cout << filename << std::endl;
  play_file = true;
  ifs.open( filename.c_str() );
  if( ifs.fail() ){
    std::cout << "Failed to open file " << filename << std::endl;
  }

}

void suture_rtt::setRCM( const std::vector<double>& xyz ){

  if( ik_solv != NULL ){ delete ik_solv; }

  rcm = KDL::Vector( xyz[0], xyz[1], xyz[2] );
  ik_solv = new KDL::ChainIkSolverPos_RCM( chain, "L7", rcm );

  std::cout << "setRCM: ";
  print( rcm );


  KDL::Frame kdlRt;
  fk_solv->JntToCart( getJointPos(), kdlRt );

  tf::Transform tfRt;
  tf::transformKDLToTF( kdlRt, tfRt );
  vw.linear.x  = vw.linear.y  = vw.linear.z  = 0.0;
  vw.angular.x = vw.angular.y = vw.angular.z = 0.0;
  vwmax.linear.x  = vwmax.linear.y  = vwmax.linear.z  = 0.005;
  vwmax.angular.x = vwmax.angular.y = vwmax.angular.z = 0.001;
  trajectory.InitializeMode( tfRt, vw, vwmax );

  KDL::Frame kdlRts = kdlRt * KDL::Frame( KDL::Vector(0.0, 0.0, -0.1) );
  tf::Transform tfRts;
  tf::transformKDLToTF( kdlRts, tfRts );
  trajectory.Push( tfRts, vw );

  init_rcm = true;

}

/**
   Read tool telemetry
*/
suture_rtt::Errno suture_rtt::readTool(){

  sensor_msgs::JointState js;
  if( port_tool_msr_jointstate.read( js ) == RTT::NewData ){
    unsigned int  N = chain.getNrOfJoints() - 7;
    for( unsigned int i=0; i<N; i++ ){
      msr_joint_pos(i+7) = js.position[i]*enc_to_rad;
      msr_joint_vel(i+7) = js.velocity[i]*enc_to_rad;
      msr_joint_trq(i+7) = js.effort[i];
    }
  }
  //print( msr_joint_pos );
  readToolWrench();
  return suture_rtt::ESUCCESS;
}

suture_rtt::Errno suture_rtt::readToolWrench(){
  if( port_tool_msr_cart_ft.connected() ){
    geometry_msgs::WrenchStamped msg_ft;
    if( port_tool_msr_cart_ft.read( msg_ft ) == RTT::NewData ){
      KDL::Vector f( msg_ft.wrench.force.x,
		     msg_ft.wrench.force.y,
		     msg_ft.wrench.force.z );
      KDL::Vector t( msg_ft.wrench.torque.x,
		     msg_ft.wrench.torque.y,
		     msg_ft.wrench.torque.z );
      msr_cart_ft = KDL::Wrench( f, t );
    }
  }
  return suture_rtt::ESUCCESS;
}


/**
   Read IIWA telemetry
*/
suture_rtt::Errno suture_rtt::readIIWA(){
  readIIWAJointsPos();
  readIIWAJointsVel();
  readIIWAJointsTrq();
  readIIWACartPos();
  readIIWAWrench();
  return suture_rtt::ESUCCESS;
}

suture_rtt::Errno suture_rtt::readIIWAJointsPos(){
  if( port_iiwa_msr_joint_pos.connected() ){
    iiwa_msgs::JointPosition msr_q;
    if( port_iiwa_msr_joint_pos.read( msr_q ) == RTT::NewData ){
      msr_joint_pos(0) = msr_q.position.a1;
      msr_joint_pos(1) = msr_q.position.a2;
      msr_joint_pos(2) = msr_q.position.a3;
      msr_joint_pos(3) = msr_q.position.a4;
      msr_joint_pos(4) = msr_q.position.a5;
      msr_joint_pos(5) = msr_q.position.a6;
      msr_joint_pos(6) = msr_q.position.a7;
    }
  }
  return suture_rtt::ESUCCESS;
}
  

suture_rtt::Errno suture_rtt::readIIWAJointsVel(){
  if( port_iiwa_msr_joint_vel.connected() ){
    iiwa_msgs::JointVelocity msr_qd;
    if( port_iiwa_msr_joint_vel.read( msr_qd ) == RTT::NewData ){
      msr_joint_vel(0) = msr_qd.velocity.a1;
      msr_joint_vel(1) = msr_qd.velocity.a2;
      msr_joint_vel(2) = msr_qd.velocity.a3;
      msr_joint_vel(3) = msr_qd.velocity.a4;
      msr_joint_vel(4) = msr_qd.velocity.a5;
      msr_joint_vel(5) = msr_qd.velocity.a6;
      msr_joint_vel(6) = msr_qd.velocity.a7;
    }
  }
  return  suture_rtt::ESUCCESS;
}

suture_rtt::Errno suture_rtt::readIIWAJointsTrq(){
  if( port_iiwa_msr_joint_trq.connected() ){
    iiwa_msgs::JointTorque msr_trq;
    if( port_iiwa_msr_joint_trq.read( msr_trq ) == RTT::NewData ){
      msr_joint_trq(0) = msr_trq.torque.a1;
      msr_joint_trq(1) = msr_trq.torque.a2;
      msr_joint_trq(2) = msr_trq.torque.a3;
      msr_joint_trq(3) = msr_trq.torque.a4;
      msr_joint_trq(4) = msr_trq.torque.a5;
      msr_joint_trq(5) = msr_trq.torque.a6;
      msr_joint_trq(6) = msr_trq.torque.a7;
    }
  }
  return suture_rtt::ESUCCESS;
}

suture_rtt::Errno suture_rtt::readIIWACartPos(){
  if( port_iiwa_msr_cart_pos.connected() ){
    geometry_msgs::PoseStamped msg_pos;
    if( port_iiwa_msr_cart_pos.read( msg_pos ) == RTT::NewData ){ 
      //tf::poseMsgToKDL( msg_pos.pose, msr_cart_pos );
    }
  }
  return suture_rtt::ESUCCESS;
}

suture_rtt::Errno suture_rtt::readIIWAWrench(){
  if( port_iiwa_msr_cart_ft.connected() ){
    geometry_msgs::WrenchStamped msg_ft;
    if( port_iiwa_msr_cart_ft.read( msg_ft ) == RTT::NewData ){
      KDL::Vector f( msg_ft.wrench.force.x,
		     msg_ft.wrench.force.y,
		     msg_ft.wrench.force.z );
      KDL::Vector t( msg_ft.wrench.torque.x,
		     msg_ft.wrench.torque.y,
		     msg_ft.wrench.torque.z );
      msr_cart_ft = KDL::Wrench( f, t );
    }
  }
  return suture_rtt::ESUCCESS;
}



suture_rtt::Errno suture_rtt::publishJointState() {

  sensor_msgs::JointState js;
  js.header.stamp = ros::Time::now();

  for( unsigned int i=0; i<chain.getNrOfJoints(); i++ ){

    char buffer[32];
    // assume joint names J[0-10] which might differ from URDF file :/
    sprintf( buffer, "iiwa_joint_%d", i+1 );

    js.name.push_back( std::string( buffer ) );
    js.position.push_back( msr_joint_pos(i) );
    js.velocity.push_back( msr_joint_vel(i) ); 
    js.effort.push_back( msr_joint_trq(i) );

  }

  port_msr_jointstate.write( js );

  return suture_rtt::ESUCCESS;
}


void suture_rtt::setJointVel( const KDL::JntArray& kdlqd ){
  if( kdlqd.rows() == chain.getNrOfJoints() ){
    iiwa_msgs::JointVelocity iiwa_qd;
    iiwa_qd.velocity.a1 = kdlqd(0);
    iiwa_qd.velocity.a2 = kdlqd(1);
    iiwa_qd.velocity.a3 = kdlqd(2);
    iiwa_qd.velocity.a4 = kdlqd(3);
    iiwa_qd.velocity.a5 = kdlqd(4);
    iiwa_qd.velocity.a6 = kdlqd(5);
    iiwa_qd.velocity.a7 = kdlqd(6);
    port_iiwa_cmd_joint_vel.write( iiwa_qd );
  }  
}

void suture_rtt::setJointPosVel( const KDL::JntArray& kdlq,
			       const KDL::JntArray& kdlqd ){

  if( kdlq.rows() == chain.getNrOfJoints() ){
    iiwa_msgs::JointPositionVelocity iiwa_qqd;
    iiwa_qqd.header.stamp = ros::Time::now();
    iiwa_qqd.position.a1 = kdlq(0);
    iiwa_qqd.position.a2 = kdlq(1);
    iiwa_qqd.position.a3 = kdlq(2);
    iiwa_qqd.position.a4 = kdlq(3);
    iiwa_qqd.position.a5 = kdlq(4);
    iiwa_qqd.position.a6 = kdlq(5);
    iiwa_qqd.position.a7 = kdlq(6);

    iiwa_qqd.velocity.a1 = kdlqd(0);
    iiwa_qqd.velocity.a2 = kdlqd(1);
    iiwa_qqd.velocity.a3 = kdlqd(2);
    iiwa_qqd.velocity.a4 = kdlqd(3);
    iiwa_qqd.velocity.a5 = kdlqd(4);
    iiwa_qqd.velocity.a6 = kdlqd(5);
    iiwa_qqd.velocity.a7 = kdlqd(6);
    
    port_iiwa_cmd_joint_pos_vel.write( iiwa_qqd );

    sensor_msgs::JointState js;
    js.header.stamp = iiwa_qqd.header.stamp;
    
    js.name.resize( 6, "inactive" );
    js.position.resize( 6, 0.0 );
    js.velocity.resize( 6, 0.0 );
    js.effort.resize( 6, 0.0 );
    
    int N = kdlq.rows() - 7;
    
    for( int i=0; i<N; i++){
      js.name[i] = std::string( "position" );
      js.position[i] = kdlq(i+7) * rad_to_enc;
    }
    port_tool_cmd_jointstate.write( js );
  }

}

void suture_rtt::setJointPos( const KDL::JntArray& kdlq ){
  
  if( kdlq.rows() == chain.getNrOfJoints() ){
    iiwa_msgs::JointPosition iiwa_q;
    iiwa_q.header.stamp = ros::Time::now();
    iiwa_q.position.a1 = kdlq(0);
    iiwa_q.position.a2 = kdlq(1);
    iiwa_q.position.a3 = kdlq(2);
    iiwa_q.position.a4 = kdlq(3);
    iiwa_q.position.a5 = kdlq(4);
    iiwa_q.position.a6 = kdlq(5);
    iiwa_q.position.a7 = kdlq(6);
    port_iiwa_cmd_joint_pos.write( iiwa_q );

    sensor_msgs::JointState js;
    js.header.stamp = iiwa_q.header.stamp;
    
    js.name.resize( 6, "inactive" );
    js.position.resize( 6, 0.0 );
    js.velocity.resize( 6, 0.0 );
    js.effort.resize( 6, 0.0 );
    
    int N = kdlq.rows() - 7;
    
    for( int i=0; i<N; i++){
      js.name[i] = std::string( "position" );
      js.position[i] = kdlq(i+7) * rad_to_enc;
    }
    port_tool_cmd_jointstate.write( js );
  }
}

void suture_rtt::setCartXYZ( const KDL::Vector& xyz ){

  geometry_msgs::PoseStamped msgRt;
  tf::poseKDLToMsg( msr_cart_pos, msgRt.pose );
  tf::pointKDLToMsg( xyz, msgRt.pose.position );

  msgRt.header.stamp = ros::Time::now();
  port_iiwa_cmd_cart_pos.write( msgRt );

}

void suture_rtt::setCartFrame( const KDL::Frame& kdlRts ){

  KDL::Frame kdlRt;
  fk_solv->JntToCart( getJointPos(), kdlRt );

  tf::Transform tfRt;
  tf::transformKDLToTF( kdlRt, tfRt );
  vw.linear.x  = vw.linear.y  = vw.linear.z  = 0.0;
  vw.angular.x = vw.angular.y = vw.angular.z = 0.0;
  vwmax.linear.x  = vwmax.linear.y  = vwmax.linear.z  = 0.01;
  vwmax.angular.x = vwmax.angular.y = vwmax.angular.z = 0.01;
  trajectory.InitializeMode( tfRt, vw, vwmax );

  tf::Transform tfRts;
  tf::transformKDLToTF( kdlRts, tfRts );
  trajectory.Push( tfRts, vw );


  //geometry_msgs::PoseStamped msgRt;
  //tf::poseKDLToMsg( Rt, msgRt.pose );

  //msgRt.header.stamp = ros::Time::now();
  //port_iiwa_cmd_cart_pos.write( msgRt );

}

suture_rtt::Errno suture_rtt::lookupTransform( const std::string& ref, 
					   const std::string& tgt,
					   KDL::Frame& kdlRt ){
  
  try{
    geometry_msgs::TransformStamped msgRt;
    msgRt = lookup_tf( ref, tgt );
    
    tf::Transform tfRt;
    tf::transformMsgToTF( msgRt.transform, tfRt );
    tf::transformTFToKDL( tfRt, kdlRt );
  }
  catch( std::exception& e ){ 
    std::cout << e.what() << std::endl; 
    return suture_rtt::EFAILURE; 
  }

  return suture_rtt::ESUCCESS;
}

void suture_rtt::print( const KDL::Twist& vw, const std::string& s ){
  std::ios::fmtflags flags = std::cout.setf( std::ios::fixed );
  std::streamsize precision = std::cout.precision(5);
  std::cout << s;
  std::cout << std::setw( 13 ) << vw.vel.x();
  std::cout << std::setw( 13 ) << vw.vel.y();
  std::cout << std::setw( 13 ) << vw.vel.z();
  std::cout << std::setw( 13 ) << vw.rot.x();
  std::cout << std::setw( 13 ) << vw.rot.y();
  std::cout << std::setw( 13 ) << vw.rot.z() << std::endl;
  std::cout.precision( precision );
  std::cout.setf( flags );
}

void suture_rtt::print( const KDL::Frame& Rt, const std::string& s ){
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

void suture_rtt::print( const KDL::JntArray& q, const std::string& s ){
  std::ios::fmtflags flags = std::cout.setf( std::ios::fixed );
  std::streamsize precision = std::cout.precision(5);
  std::cout << s;
  for( size_t i=0; i<q.rows(); i++ )
  {std::cout << std::setw( 13 ) << q( i );}
  std::cout << std::endl;

  std::cout.precision( precision );
  std::cout.setf( flags );
}

void suture_rtt::print( const KDL::Vector& v, const std::string& s ){
  std::ios::fmtflags flags = std::cout.setf( std::ios::fixed );
  std::streamsize precision = std::cout.precision(5);

  std::cout << s;
  std::cout << std::setw( 13 ) << v.x();
  std::cout << std::setw( 13 ) << v.y();
  std::cout << std::setw( 13 ) << v.z();
  std::cout << std::endl;

  std::cout.precision( precision );
  std::cout.setf( flags );
}

void suture_rtt::print( const KDL::Wrench& ft, const std::string& s ){
  std::cout << s;
  print( ft.force, "force: " );
  print( ft.torque, "torque: " );
  std::cout << std::endl;
}

void suture_rtt::ReadMarkers(){

  sensor_msgs::PointCloud2 ros_markers;
  if( port_markers.read( ros_markers ) == RTT::NewData && plan.empty() ){

    // markers are in the raytrix frame
    KDL::Frame kdl_Rt;
   // if( EFAILURE == lookupTransform( "world", "intel", kdl_Rt ) ){

     // RTT::log(RTT::Error) << "Failed to query tf suture-raytrix" << RTT::endlog();
   // }
    //else{

      //pcl::PointCloud<pcl::PointXYZI> pcl_markers;
      //pcl::fromROSMsg( ros_markers, pcl_markers );
      
     // Eigen::Affine3d eig_Rt;
     // tf::transformKDLToEigen( kdl_Rt, eig_Rt );
      
      //pcl::transformPointCloud( pcl_markers, markers, eig_Rt );
      
      // untanggle the repeats
 
      pcl::fromROSMsg( ros_markers, markers );
      pcl::PointXYZI p1 = markers[0];
      pcl::PointXYZI p2 = markers[1];
      
      float x1 = p1.x;
      float x2 = p2.x;
      
      float y1 = p1.y;
      float y2 = p2.y;

      float z1 = p1.z;
      float z2 = p2.z;

      if( y1 < y2 ){
	float tmp;
	tmp = x1; x1 = x2; x2 = tmp;
	tmp = y1; y1 = y2; y2 = tmp;
	tmp = z1; z1 = z2; z2 = tmp;
      }

      
      int NUM_STITCHES = sqrt( (x2-x1)*(x2-x1) +
			       (y2-y1)*(y2-y1) +
			       (z2-z1)*(z2-z1) ) / 0.005;
	
      float dx=(x2-x1)/(NUM_STITCHES+1);
      float dy=(y2-y1)/(NUM_STITCHES+1);
      float dz=(z2-z1)/(NUM_STITCHES+1);
      
      std::cout << std::endl
		<< "Marker 1: "
		<< std::setw(15) << x1 << std::setw(15) << y1 << std::setw(15) << z1
		<< std::endl
		<< "Marker 2: "
		<< std::setw(15) << x2 << std::setw(15) << y2 << std::setw(15) << z2
		<< std::endl;

      // this is for the start position (all stitches shouldn't be started
      // from this)
      // ... but they are
//      KDL::Frame Rt;
//      fk_solv->JntToCart( GetJointPos(), Rt );
      
      for( int i=0; i<NUM_STITCHES; i++ ){
	pcl::PointXYZI pi;
	pi.x = x1 + dx*(i+1);
	pi.y = y1 + dy*(i+1);
	pi.z = z1 + dz*(i+1);

	std::cout << std::endl
		<< "waypoint: "
		<< std::setw(15) << pi.x << std::setw(15) << pi.y << std::setw(15) << pi.z
		<< std::endl;

	plan.push_back( pi );
	pi.x = x1 + dx*(i+1)-0.003;
	pi.y = y1 + dy*(i+1)+0.01;
	pi.z = z1 + dz*(i+1)-0.008;
	// what are these?!
	if( 0 < i ){
	  pi.x -= 0.002;
	  pi.z -= 0.004;
	}
	
	
//	KDL::Frame Rts( Rt );
//	Rts.p.x( pi.x );
//	Rts.p.y( pi.y );
//	Rts.p.z( pi.z );
	
	//StitchContact* stitch = new StitchContact( Rts );
	//stitch->Start( Rt );
	//suture.push_back( stitch );
//	frames.push_back( Rts );
	
//	if( i == (NUM_STITCHES-1) ){
	  //StitchContact* stitch = new StitchContact( Rts );
	  //stitch->Start( Rt );
	  //suture.push_back( stitch );
//	  frames.push_back( Rts );
//	}
      }
      go = true;
      
      //StitchContact* stitch = new StitchContact( frames.front() );
      //stitch->Start( Rt );
      //suture.push_back( stitch );
      
    //}
  }
}



ORO_CREATE_COMPONENT( suture_rtt )
