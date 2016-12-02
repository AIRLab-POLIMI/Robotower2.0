/* COPYRIGHT (c) 2016 Nova Labs SRL
 *
 * All rights reserved. All use of this software and documentation is
 * subject to the License Agreement located in the file LICENSE.
 */

#pragma once

#include <core/mw/Publisher.hpp>
#include <core/mw/Subscriber.hpp>
#include <core/mw/CoreNode.hpp>
#include <core/os/Callback.hpp>

#include <ModuleConfiguration.hpp>

#include <core/triskar_kinematics/InverseConfiguration.hpp>
#include <core/triskar_msgs/Velocity.hpp>
#include <core/actuator_msgs/Setpoint_f32.hpp>

namespace core {
namespace triskar_kinematics {
class Inverse:
   public mw::CoreNode,
   public mw::CoreConfigurable<InverseConfiguration>
{
public:
   Inverse(
      const char*          name,
      os::Thread::Priority priority = os::Thread::PriorityEnum::NORMAL
   );
   virtual
   ~Inverse();

private:
   bool
   onPrepareMW();

   bool
   onLoop();

   static bool
   callback(
      const triskar_msgs::Velocity& msg,
      void*                                    context
   );


private:
   mw::Subscriber<triskar_msgs::Velocity, 5> _subscriber;
   mw::Publisher<actuator_msgs::Setpoint_f32> _wheel_publisher[3];
};
}
}
