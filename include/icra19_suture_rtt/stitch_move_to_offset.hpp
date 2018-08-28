#ifndef _STITCH_MOVE_TO_OFFSET_HPP_
#define _STITCH_MOVE_TO_OFFSET_HPP_

#include <icra19_suture_rtt/stitch_base.hpp>

//! Pure translation to global offset read from star_config
//! Goal is to move the tool out of the vision by 'Jog'ing in the base frame
class StitchMoveToOffset : public StitchBase {
  
protected:
  
  double contact_f;   // contact force
  double contact_d;   // contact distance
  
public:
  
  // Simple move without stitch
  StitchMoveToOffset( const KDL::Frame& Rts,
		      double cf = 0.5,
		      double cd = 0.01,
		      double tf = 4.0,
		      double td = 0.2 );
  
  ~StitchMoveToOffset();
  
  virtual State Evaluate( KDL::Frame& Rt,
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
  
  virtual void Start( KDL::Frame& Rt );
  virtual void Approach();
  virtual void Hover();
};


#endif
