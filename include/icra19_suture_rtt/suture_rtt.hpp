
//#include <std// Copyright  (C)  2013 Simon Leonard

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


#ifndef _SUTURE_RTT_HPP_
#define _SUTURE_RTT_HPP_


#include <string>
#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include <rtt/TaskContext.hpp>
#include <rtt/InputPort.hpp>
#include <rtt/OutputPort.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/os/TimeService.hpp>

#include <iiwa_msgs/JointPosition.h>
#include <iiwa_msgs/JointVelocity.h>
#include <iiwa_msgs/JointPositionVelocity.h>
#include <iiwa_msgs/JointTorque.h>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainiksolver.hpp>

#include <sensor_msgs/JointState.h>

#include <star_rtt/trajectory.hpp>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h> 
#include <geometry_msgs/Twist.h> 
#include <geometry_msgs/TwistStamped.h> 
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>

#include<icra19_suture_rtt/stitch_contact.hpp>

class suture_rtt : public RTT::TaskContext {
  
public:
  
  enum Errno{ ESUCCESS, EFAILURE };
  
private:
  
  RTT::os::TimeService* ts;
  RTT::os::TimeService::ticks ticks;

  /**
     Telemetry ports
  */

  //! Port to get the joint positions
  RTT::InputPort< iiwa_msgs::JointPosition > port_iiwa_msr_joint_pos;
  
  //! Port to get the joint velocities
  RTT::InputPort< iiwa_msgs::JointVelocity > port_iiwa_msr_joint_vel;
  
    //! Port to get the joint torques
  RTT::InputPort< iiwa_msgs::JointTorque > port_iiwa_msr_joint_trq;
  
  //! Port to get the position/orientation from the IIWA
  RTT::InputPort< geometry_msgs::PoseStamped > port_iiwa_msr_cart_pos;

  //! Port to get the Cartesian force/torque from the IIWA
  RTT::InputPort< geometry_msgs::WrenchStamped > port_iiwa_msr_cart_ft;


  /**
     Command ports to IIWA
  */

  //! Port to set the joint positions
  RTT::OutputPort< iiwa_msgs::JointPosition > port_iiwa_cmd_joint_pos;

  //! Port to set the joint positions
  RTT::OutputPort< iiwa_msgs::JointVelocity > port_iiwa_cmd_joint_vel;

    //! Port to set the joint positions
  RTT::OutputPort< iiwa_msgs::JointPositionVelocity > port_iiwa_cmd_joint_pos_vel;

  //! Port to set the position/orientation of the IIWA
  RTT::OutputPort< geometry_msgs::PoseStamped > port_iiwa_cmd_cart_pos;
  
  KDL::JntArray msr_joint_pos;
  KDL::JntArray msr_joint_vel;
  KDL::JntArray msr_joint_trq;
  KDL::Frame    msr_cart_pos;
  KDL::Wrench   msr_cart_ft;    
 
  suture_rtt::Errno readIIWA();  
  suture_rtt::Errno readIIWAJointsPos();
  suture_rtt::Errno readIIWAJointsVel();
  suture_rtt::Errno readIIWAJointsTrq();
  suture_rtt::Errno readIIWACartPos();
  suture_rtt::Errno readIIWAWrench();
  
  //! Port to publish the joint states
  RTT::OutputPort< sensor_msgs::JointState > port_msr_jointstate;

  //! Port to set joint state command
  RTT::InputPort< sensor_msgs::JointState > port_cmd_jointstate;

  suture_rtt::Errno publishJointState();


  /**
     TF hooks
  */
  RTT::OperationCaller<geometry_msgs::TransformStamped(const std::string&,
						       const std::string&)>
  lookup_tf;
  suture_rtt::Errno lookupTransform( const std::string& ref, 
				   const std::string& tgt,
				   KDL::Frame& kdlRt );


  /**
     Methods for setting the RCM
  */
  KDL::Vector rcm;
  bool init_rcm;
// added HS
  bool test_traj_initialized;
  bool test_stitch_defined;
  StitchContact* test_stitch;

  // use this to get the XYZ coordinates of the TCP
  std::vector<double> getXYZ( const std::vector<double>& xyz );

  // Set the RCM to the XYZ coordinates
  void setRCM( const std::vector<double>& xyz );

protected:

  /** 
      Tool ports
  */
  RTT::InputPort< sensor_msgs::JointState >      port_tool_msr_jointstate;
  RTT::InputPort< geometry_msgs::WrenchStamped > port_tool_msr_cart_ft;
  RTT::OutputPort< sensor_msgs::JointState >     port_tool_cmd_jointstate;

  suture_rtt::Errno readTool();
  suture_rtt::Errno readToolWrench();
  
  double enc_to_rad;
  double rad_to_enc;

  // used to cancel roll//pitch angles
  KDL::Frame cancelRoll( const KDL::Frame& Rt0n );
  KDL::Frame cancelPitch( const KDL::Frame& Rt0n );
  
  Trajectory trajectory;
  geometry_msgs::Twist vw;
  geometry_msgs::Twist vwmax;

  
  /**
     Robot description and kinematics
  */
  std::string urdf_name;
  std::string robot_description_name;
  std::string robot_description;
  std::string ref_frame;
  std::string tcp_frame;

  KDL::Chain chain;
  KDL::ChainFkSolverPos* fk_solv;
  KDL::ChainIkSolverVel* vw_solv;
  KDL::ChainIkSolverPos* ik_solv;


  // Accessors
  KDL::JntArray getJointPos() const { return msr_joint_pos; }
  KDL::JntArray getJointVel() const { return msr_joint_vel; }
  KDL::JntArray getJointTrq() const { return msr_joint_trq; }
  KDL::Vector   getCartPos()  const { return msr_cart_pos.p; }
  KDL::Rotation getCartRot()  const { return msr_cart_pos.M; }
  KDL::Frame    getCartFrame()const { return msr_cart_pos; }
  KDL::Wrench   getCartFT()   const { return msr_cart_ft; }

  bool play_file;
  std::ifstream ifs;
  void playFile( const std::string& filename );
  
  void setJointPos( const KDL::JntArray& kdlq );
  void setJointVel( const KDL::JntArray& kdlqd );
  void setJointPosVel( const KDL::JntArray& kdlq,
		       const KDL::JntArray& kdlqd );
  void setCartXYZ( const KDL::Vector& xyz );
  void setCartFrame( const KDL::Frame& Rt );
  
  // RTT stuff
  virtual bool configureHook();
  virtual bool startHook();
  virtual void updateHook();
  virtual void stopHook();
  virtual void cleanupHook();

  virtual suture_rtt::Errno evaluate();
  
public:
  
  suture_rtt( const std::string& name );
  virtual ~suture_rtt();



  void print( const KDL::Twist& vw, const std::string& s=std::string() );
  void print( const KDL::Frame& Rt, const std::string& s=std::string() );
  void print( const KDL::JntArray& q, const std::string& s=std::string() );
  void print( const KDL::Vector& v, const std::string& s=std::string() );
  void print( const KDL::Wrench& ft, const std::string& s=std::string() );
  
};

#endif

