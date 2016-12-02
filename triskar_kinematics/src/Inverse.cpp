/* COPYRIGHT (c) 2016 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#include <Module.hpp>

#include <core/triskar_kinematics/Inverse.hpp>
#include <core/utils/math/Constants.hpp>
#include <core/utils/math/Conversions.hpp>

#include <core/triskar_msgs/Velocity.hpp>
#include <core/actuator_msgs/Setpoint_f32.hpp>

#include <cmath>

using namespace core::utils::math::constants;
using namespace std;

namespace core {
namespace triskar_kinematics {
Inverse::Inverse(
   const char*          name,
   os::Thread::Priority priority
) :
   CoreNode::CoreNode(name, priority),
   CoreConfigurable::CoreConfigurable(name)
{
   _workingAreaSize = 1024;
}

Inverse::~Inverse()
{
   teardown();
}

bool
Inverse::onPrepareMW()
{
   _subscriber.set_callback(Inverse::callback);

   this->subscribe(_subscriber, configuration().velocity_input);
   this->advertise(_wheel_publisher[0], configuration().output_0);
   this->advertise(_wheel_publisher[1], configuration().output_1);
   this->advertise(_wheel_publisher[2], configuration().output_2);

   return true;
}

bool
Inverse::onLoop()
{
   if (!this->spin(ModuleConfiguration::SUBSCRIBER_SPIN_TIME)) {}

   return true;
}

bool
Inverse::callback(
   const triskar_msgs::Velocity& msg,
   void*                                    context
)
{
   Inverse* _this = static_cast<Inverse*>(context);

   const float L = _this->configuration().center_distance;
   const float R = _this->configuration().wheel_radius;

   actuator_msgs::Setpoint_f32* _speed[3];

   float dx = msg.linear[0];
   float dy = msg.linear[1];
   float dphi = msg.angular;


   /// DO THE MATH
   float dtheta[3];

   dtheta[0] = (sin(pi<float>()/3.0)*dy + (cos(pi<float>()/3.0)+1)*dx + L*dphi*sin(pi<float>()/3.0))
		/ (R*(2.0*cos(pi<float>()/3.0)+2.0)*sin(pi<float>()/3.0));
   dtheta[1] = (sin(pi<float>()/3.0)*dy - (cos(pi<float>()/3.0)+1)*dx + L*dphi*sin(pi<float>()/3.0))
		/ (R*(2.0*cos(pi<float>()/3.0)+2.0)*sin(pi<float>()/3.0));
   dtheta[2] = (-dy + L*cos(pi<float>()/3.0)*dphi)/(R*(cos(pi<float>()/3.0)+1));

   for(unsigned int i = 0; i < 3; i++)
   {
	   if (_this->_wheel_publisher[i].alloc(_speed[i])) {
	      /// PUBLISH THE RESULTS
	      _speed[i]->value = dtheta[i];

	      if (!_this->_wheel_publisher[i].publish(_speed[i]))
	      {
	         return false;
	      }
	   }
   }

   return true;
} // Inverse::callback
}
}
