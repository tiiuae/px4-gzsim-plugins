/*
 * Copyright 2015  Aurelien Roy
 *
 * This file is a modified version of github.com/AurelienRoy/ardupilot_sitl_gazebo_plugin.git
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @brief Parachute Plugin
 *
 * This plugin simulates parachute deployment
 *
 * @author Aurelien Roy  <aurroy@hotmail.com>
 */

#ifndef _GAZEBO_PARACHUTE_PLUGIN_HH_
#define _GAZEBO_PARACHUTE_PLUGIN_HH_

#include <common.h>
#include <math.h>
#include <sdf/sdf.hh>

#include <gz/common/Filesystem.hh>
#include <gz/common5/gz/common.hh>
#include <gz/math.hh>
#include <gz/msgs.hh>
#include <gz/msgs/actuators.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Events.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>

// #include <Odometry.pb.h>

// typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;

static const std::string KDefaultParachuteTriggerTopic = "/servo_7";

namespace parachute_plugin {

class ParachutePlugin : public gz::sim::System,
                        public gz::sim::ISystemConfigure,
                        public gz::sim::ISystemPreUpdate,
                        public gz::sim::ISystemPostUpdate {
public:
  ParachutePlugin() = default;

  void Configure(const gz::sim::Entity& _entity, const std::shared_ptr<const sdf::Element>& _sdf,
                 gz::sim::EntityComponentManager& _ecm, gz::sim::EventManager& _eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm) override;

  void PostUpdate(const gz::sim::UpdateInfo& _info,
                  const gz::sim::EntityComponentManager& _ecm) override;

private:
  /// \brief Loads parachute model
  /// \note takes a non-const ECM because model insertion and component creation
  /// require modifying the entity/component store.
  void LoadParachute(gz::sim::EntityComponentManager&);

  /// \brief Attach the parachute to the model
  /// \param parachute_model Parachute model to attach to the vehicle
  void AttachParachute(gz::sim::Entity, gz::sim::EntityComponentManager&);

  /// \brief Callback for subscribing to motor commands
  /// \param rot_velocities Motor command velocity commanded from firmware
  void ParachuteServoCallback(const gz::msgs::Double&);

  gz::sim::Entity model_entity_;
  gz::sim::Entity world_;
  gz::sim::Entity parachute_entity_{gz::sim::kNullEntity};
  gz::sim::Entity parachute_link_{gz::sim::kNullEntity};
  gz::sim::Entity base_link_entity_{gz::sim::kNullEntity};

  // physics::WorldPtr world_;
  // event::ConnectionPtr update_connection_;

  bool active_servo = false;
  bool attached_parachute_ = false;
  bool spawn_requested_ = false;
  std::string world_name_;
  // double max_rot_velocity_ = 3500;   ///< Clip maximum motor velocity
  // double ref_motor_rot_vel_ = 0;     ///< Reference motor velocity
  // double terminate_rot_vel_ = 1700-; ///< Motor velocity command in flight termination step
  // int motor_number_;

  std::shared_ptr<gz::transport::Node> node_;

}; // class GAZEBO_VISIBLE ParachutePlugin
} // namespace parachute_plugin
#endif // _GAZEBO_PARACHUTE_PLUGIN_HH_