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

#include "gazebo_parachute_plugin.h"

namespace parachute_plugin {

void ParachutePlugin::Configure(const gz::sim::Entity& _entity,
                                const std::shared_ptr<const sdf::Element>& _sdf,
                                gz::sim::EntityComponentManager& _ecm,
                                gz::sim::EventManager& _eventMgr) {

  // Prev_Args: physics::ModelPtr model, sdf::ElementPtr sdf

  this->node_ = std::make_shared<gz::transport::Node>();

  this->model_entity_ = _entity;
  this->world_ = gz::sim::worldEntity(_ecm);

  // Get world name for service topic
  auto world_name_comp = _ecm.Component<gz::sim::components::Name>(this->world_);
  if (world_name_comp) {
    this->world_name_ = world_name_comp->Data();
  }

  // Find the base_link entity
  auto model = gz::sim::Model(_entity);
  this->base_link_entity_ = model.LinkByName(_ecm, "base_link");
  if (this->base_link_entity_ == gz::sim::kNullEntity) {
    gzwarn << "[gazebo_parachute_plugin] base_link not found, will use model entity\n";
  } else {
    gzdbg << "[gazebo_parachute_plugin] Found base_link entity\n";
  }

  // Get model name for topic subscription
  auto model_name_comp = _ecm.Component<gz::sim::components::Name>(this->model_entity_);
  std::string model_name = model_name_comp ? model_name_comp->Data() : "unknown";

  // Construct the model-specific topic: /model/{model_name}/command/motor_speed
  std::string parachute_servo_topic = "/model/" + model_name + KDefaultParachuteTriggerTopic;

  gzdbg << "[gazebo_parachute_plugin] Subscribing to topic: " << parachute_servo_topic << "\n";

  this->node_->Subscribe<gz::msgs::Double>(parachute_servo_topic,
                                           [this](const gz::msgs::Double& msg) -> void {
                                             //  std::cout << msg.data() << std::endl;
                                             this->ParachuteServoCallback(msg);
                                           });
}

void ParachutePlugin::PreUpdate(const gz::sim::UpdateInfo& _info,
                                gz::sim::EntityComponentManager& _ecm) {

  // Spawn parachute only when all motors are active
  if (this->active_servo && this->parachute_entity_ == gz::sim::kNullEntity &&
      !this->spawn_requested_) {
    std::cout << "Triggering Load of parachute - all motors active" << std::endl;
    LoadParachute(_ecm);
  }

  // Check if parachute has spawned after we requested it
  if (this->spawn_requested_ && !this->attached_parachute_) {
    auto parachute_entity = _ecm.EntityByComponents(gz::sim::components::Name("parachute_deployed"),
                                                    gz::sim::components::Model());

    if (parachute_entity != gz::sim::kNullEntity) {
      AttachParachute(parachute_entity, _ecm);
    }
  }

  if (!this->active_servo && this->attached_parachute_) {
    gzdbg << "[gazebo_parachute_plugin] Servo inactive, detaching and removing parachute\n";

    // Remove the DetachableJoint component to detach the parachute
    if (this->parachute_entity_ != gz::sim::kNullEntity) {
      _ecm.RemoveComponent<gz::sim::components::DetachableJoint>(this->parachute_entity_);

      // Request entity removal via service
      gz::msgs::Entity entityMsg;
      entityMsg.set_id(this->parachute_entity_);

      std::string remove_service = "/world/" + this->world_name_ + "/remove";
      gz::msgs::Boolean response;
      bool result = false;
      unsigned int timeout = 5000;
      bool executed = this->node_->Request(remove_service, entityMsg, timeout, response, result);

      if (executed && result && response.data()) {
        gzdbg << "[gazebo_parachute_plugin] Parachute removed successfully\n";
      } else {
        gzwarn << "[gazebo_parachute_plugin] Failed to remove parachute entity\n";
      }
    }

    // Reset state variables
    this->parachute_entity_ = gz::sim::kNullEntity;
    this->parachute_link_ = gz::sim::kNullEntity;
    this->attached_parachute_ = false;
    this->spawn_requested_ = false;
  }
}

// void ParachutePlugin::OnUpdate(const common::UpdateInfo&) {

//   physics::ModelPtr parachute_model = GetModelPtr("parachute_small");
//   // Trigger parachute if flight termination
//   if (ref_motor_rot_vel_ <= terminate_rot_vel_)
//     LoadParachute();

//   if (!attached_parachute_ && parachute_model) {
//     AttachParachute(parachute_model); // Attach parachute to model
//     attached_parachute_ = true;
//   }
// }

void ParachutePlugin::ParachuteServoCallback(const gz::msgs::Double& servo_velocity) {
  // Check the Actuators message for velocity entries and whether they're active.
  if (servo_velocity.data() == 0.0f) {
    // gzdbg << "[gazebo_parachute_plugin] Actuators message has zero velocity entries\n";
    return;
  }

  bool active = (servo_velocity.data() > 0.0f or servo_velocity.data() < 0.0f) ? true : false;
  // double max_vel = 0.0;
  // for (int i = 0; i < n; ++i) {
  //   double v = rot_velocities.velocity(i);
  //   if (std::isnan(v) || v == 0.0) {
  //     active = false;
  //   }
  //   if (std::abs(v) > std::abs(max_vel))
  //     max_vel = v;
  // }

  if (active) {
    // std::cout << "[gazebo_parachute_plugin] All " << int(active) << " servo active\n";
    this->active_servo = true;
  } else {
    // std::cout << "[gazebo_parachute_plugin] Not all servo active (" << servo_velocity.data()
    //           << " )\n";
    this->active_servo = false;
  }
}

void ParachutePlugin::LoadParachute(gz::sim::EntityComponentManager& _ecm) {
  if (this->parachute_entity_ != gz::sim::kNullEntity || this->spawn_requested_)
    return;

  gzdbg << "[gazebo_parachute_plugin] Requesting parachute spawn\n";

  gz::msgs::EntityFactory factoryMsg;
  factoryMsg.set_sdf_filename("model://parachute");
  factoryMsg.set_name("parachute_deployed");

  // Set spawn position above the vehicle
  auto model_pose_opt = _ecm.ComponentData<gz::sim::components::Pose>(this->model_entity_);
  if (model_pose_opt) {
    auto spawn_pose = *model_pose_opt;
    spawn_pose.Pos().Z(spawn_pose.Pos().Z() + 0.5);
    gz::msgs::Set(factoryMsg.mutable_pose(), spawn_pose);
  }

  // Use the /world/{world_name}/create service to spawn the entity
  std::string service_name = "/world/" + this->world_name_ + "/create";
  gzdbg << "[gazebo_parachute_plugin] Calling service: " << service_name << "\n";

  gz::msgs::Boolean response;
  bool result = false;
  unsigned int timeout = 5000;
  bool executed = this->node_->Request(service_name, factoryMsg, timeout, response, result);

  if (executed && result && response.data()) {
    gzdbg << "[gazebo_parachute_plugin] Parachute spawn requested successfully\n";
    this->spawn_requested_ = true;
  } else {
    gzerr << "[gazebo_parachute_plugin] Failed to request parachute spawn. "
          << "Executed: " << executed << ", Result: " << result << "\n";
  }
}
void ParachutePlugin::AttachParachute(gz::sim::Entity parachute_entity,
                                      gz::sim::EntityComponentManager& _ecm) {

  if (parachute_entity == gz::sim::kNullEntity) {
    gzerr << "[gazebo_parachute_plugin] AttachParachute called with null entity\n";
    return;
  }

  // Get the parachute's link entity (the link is called "chute" in the parachute model)
  auto parachute_model = gz::sim::Model(parachute_entity);
  auto parachute_link = parachute_model.LinkByName(_ecm, "chute");

  if (parachute_link == gz::sim::kNullEntity) {
    gzerr << "[gazebo_parachute_plugin] Could not find 'chute' link in parachute model\n";
    return;
  }

  // Use base_link if available, otherwise use model entity as parent link
  gz::sim::Entity parent_link = (this->base_link_entity_ != gz::sim::kNullEntity)
                                    ? this->base_link_entity_
                                    : gz::sim::Model(this->model_entity_).CanonicalLink(_ecm);

  if (parent_link == gz::sim::kNullEntity) {
    gzerr << "[gazebo_parachute_plugin] Could not find parent link to attach parachute\n";
    return;
  }

  // Create a physical attachment using DetachableJoint
  // This will create a fixed joint constraint between base_link and the chute link
  gz::sim::components::DetachableJointInfo jointInfo;
  jointInfo.parentLink = parent_link;
  jointInfo.childLink = parachute_link;
  jointInfo.jointType = "fixed";

  // Store the parachute link
  this->parachute_link_ = parachute_link;

  // Create the DetachableJoint component on the parachute model entity
  _ecm.CreateComponent(parachute_entity, gz::sim::components::DetachableJoint(jointInfo));

  this->parachute_entity_ = parachute_entity;
  this->attached_parachute_ = true;

  gzdbg
      << "[gazebo_parachute_plugin] Parachute physically attached to base_link with fixed joint\n";
}

void ParachutePlugin::PostUpdate(const gz::sim::UpdateInfo& _info,
                                 const gz::sim::EntityComponentManager& _ecm) {
  // Continuously update parachute position to follow the base_link
  // This creates a kinematic attachment
  // if (this->attached_parachute_ && this->parachute_link_ != gz::sim::kNullEntity &&
  //     this->base_link_entity_ != gz::sim::kNullEntity) {

  // Get base_link world pose
  // auto base_link_pose =
  //     _ecm.ComponentData<gz::sim::components::WorldPose>(this->base_link_entity_);

  // if (base_link_pose) {
  //   // Position parachute 1m above base_link
  //   gz::math::Pose3d parachute_world_pose = *base_link_pose;
  //   parachute_world_pose.Pos().Z(parachute_world_pose.Pos().Z() + 3.0);

  //   // Note: Can't modify in PostUpdate (const ECM), this will be done via setting pose in
  //   model
  //   // The SetParentEntity should handle the kinematic attachment
  // }
  // }
}
} // namespace parachute_plugin

GZ_ADD_PLUGIN(parachute_plugin::ParachutePlugin, gz::sim::System,
              parachute_plugin::ParachutePlugin::ISystemConfigure,
              parachute_plugin::ParachutePlugin::ISystemPreUpdate,
              parachute_plugin::ParachutePlugin::ISystemPostUpdate);

GZ_ADD_PLUGIN_ALIAS(parachute_plugin::ParachutePlugin, "custom::ParachutePlugin");