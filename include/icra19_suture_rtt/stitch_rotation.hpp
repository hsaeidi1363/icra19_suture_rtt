#ifndef _STITCH_ROTATION_HPP_
#define _STITCH_ROTATION_HPP_

#include <icra19_suture_rtt/stitch_contact.hpp>

class StitchRotation : public StitchContact {

protected:
  
  KDL::JntArray dqs;
  
public:
  
  StitchRotation( const KDL::Frame& Rts,      // desired frame
		  const KDL::JntArray& dqs,
		  double cf = 0.5,
		  double cd = 0.01,
		  double tf = 4.0,
		  double td = 0.2 ); 
  
  ~StitchRotation();

  virtual StitchBase::State Evaluate( KDL::Frame& Rt,
				      KDL::Twist& vw,
				      KDL::JntArray& q,
				      KDL::JntArray& qd,
				      const KDL::Vector& f=KDL::Vector::Zero(), 
				      bool accept_stitch = false, 
				      bool repeat_stitch = false,
				      const geometry_msgs::Vector3& offset_vector  = zero3(),
				      const geometry_msgs::Vector3& contact_vector = zero3(),
				      const geometry_msgs::Vector3& contact_force  = zero3(),
				      const geometry_msgs::Vector3& tension_vector = zero3(),
				      const geometry_msgs::Vector3& tension_forice = zero3(),
				      bool accept_tension = false
				      );  
};

#endif
