#ifndef _STITCH_CONTACT_HPP
#define _STITCH_CONTACT_HPP

#include <icra19_suture_rtt/stitch_base.hpp>

//! Main class for simple contact-based stitch.
//! Subclasses are Stitch{Knot, Joint, Rotation, Slide}
class StitchContact : public StitchBase {
  
protected:
  
  double contact_f;   // contact force
  double contact_d;   // contact distance
  
public:
  
  // Contact stitch 
  StitchContact( const KDL::Frame& Rts,
		 double cf = 0.5,
		 double cd = 0.01,
		 double tf = 4.0,
		 double td = 0.1 );
  
  ~StitchContact();
  
  //void InitNewTarget( const KDL::Frame& Rt1, const KDL::Frame& Rt2 );
  
  virtual State Evaluate( KDL::Frame& Rt,
			  KDL::Twist& vw,
			  KDL::JntArray& q,
			  KDL::JntArray& qd,
			  const KDL::Vector& f = KDL::Vector::Zero(),
			  bool accept_stitch = true,
			  bool repeat_stitch = false,
			  const geometry_msgs::Vector3& offset_vector  = zero3(),
			  const geometry_msgs::Vector3& contact_vector = zero3(),
			  const geometry_msgs::Vector3& contact_force  = zero3(),
			  const geometry_msgs::Vector3& tension_vector = zero3(),
			  const geometry_msgs::Vector3& tension_force  = zero3(),
			  bool accept_tension = true
			  );
  
  virtual void Start( KDL::Frame& Rt );
  virtual void Approach();
  virtual void Hover();
  
};

#endif
