#ifndef _STITCH_KNOT_HPP_
#define _STITCH_KNOT_HPP_

#include <icra19_suture_rtt/stitch_contact.hpp>

//! StitchBase :: StitchContact :: StitchKnot 
//! Used to be called AzadKnot (now StitchKnot)
class StitchKnot : public StitchContact {
  
protected:
  
  size_t num_loops;
  size_t cur_loop;
  bool   SURGEONS_KNOT; // First a throw doubles
  
public:
  
  StitchKnot( const KDL::Frame& Rts,      // desired frame
	      size_t nloops=1,
	      double cf = 0.5,
	      double cd = 0.01,
	      double tf = 4.0,
	      double td = 0.2 ); 
  
  ~StitchKnot();
  
  //! Specialized Hover for knot
  void KnotHover();
  
  virtual StitchBase::State Evaluate( KDL::Frame& Rt, 
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
				      bool accept_tension = false
				      );
  
};

#endif
