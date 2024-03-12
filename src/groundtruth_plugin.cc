/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
 *
 */

#include "groundtruth_plugin.hh"

#include <gz/msgs/pose.pb.h>
#include <gz/msgs/time.pb.h>

#include <stack>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <sdf/Joint.hh>

#include <gz/common/Profiler.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>

#include "gz/sim/Util.hh"
#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/ChildLinkName.hh"
#include "gz/sim/components/Collision.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointType.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Model.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/ParentLinkName.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Sensor.hh"
#include "gz/sim/components/Visual.hh"
#include "gz/sim/Conversions.hh"

using namespace gz;
using namespace sim;
using namespace systems;

//////////////////////////////////////////////////
PosePublisher::PosePublisher()
{
}

//////////////////////////////////////////////////
void PosePublisher::Configure(const Entity &_entity,
                              const std::shared_ptr<const sdf::Element> &_sdf,
                              EntityComponentManager &_ecm,
                              EventManager &/*_eventMgr*/)
{
  model = Model(_entity);

  if (!model.Valid(_ecm))
  {
    gzerr << "PosePublisher plugin should be attached to a model entity. "
      << "Failed to initialize." << std::endl;
    return;
  }

  // parse optional params
  publishLinkPose = _sdf->Get<bool>("publish_link_pose",
      publishLinkPose).first;

  publishNestedModelPose =
    _sdf->Get<bool>("publish_nested_model_pose",
        publishNestedModelPose).first;

  // for backward compatibility, publish_model_pose will be set to the
  // same value as publish_nested_model_pose if it is not specified.
  publishModelPose =
    _sdf->Get<bool>("publish_model_pose",
        publishNestedModelPose).first;

  publishVisualPose =
    _sdf->Get<bool>("publish_visual_pose",
        publishVisualPose).first;

  publishCollisionPose =
    _sdf->Get<bool>("publish_collision_pose",
        publishCollisionPose).first;

  publishSensorPose =
    _sdf->Get<bool>("publish_sensor_pose",
        publishSensorPose).first;

  double updateFrequency = _sdf->Get<double>("update_frequency", -1).first;

  if (updateFrequency > 0)
  {
    std::chrono::duration<double> period{1 / updateFrequency};
    updatePeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
  }

  staticPosePublisher =
    _sdf->Get<bool>("static_publisher",
        staticPosePublisher).first;

  if (staticPosePublisher)
  {
    // update rate for static transforms. Default to same as <update_frequency>
    double staticPoseUpdateFrequency =
      _sdf->Get<double>("static_update_frequency", updateFrequency).first;

    if (staticPoseUpdateFrequency > 0)
    {
      std::chrono::duration<double> period{1 / staticPoseUpdateFrequency};
      staticUpdatePeriod =
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          period);
    }
  }

  // create publishers
  usePoseV =
    _sdf->Get<bool>("use_pose_vector_msg", usePoseV).first;

  std::string poseTopic = scopedName(_entity, _ecm) + "/pose_custom";
  poseTopic = transport::TopicUtils::AsValidTopic(poseTopic);
  if (poseTopic.empty())
  {
    poseTopic = "/pose_custom";
    gzerr << "Empty pose topic generated for pose_publisher system. "
           << "Setting to " << poseTopic << std::endl;
  }
  std::string staticPoseTopic = poseTopic + "_static";

  if (usePoseV)
  {
    posePub =
      node.Advertise<msgs::Pose_V>(poseTopic);

    if (staticPosePublisher)
    {
      poseStaticPub =
          node.Advertise<msgs::Pose_V>(
          staticPoseTopic);
    }
  }
  else
  {
    posePub =
      node.Advertise<msgs::Pose>(poseTopic);
    if (staticPosePublisher)
    {
      poseStaticPub =
          node.Advertise<msgs::Pose>(
          staticPoseTopic);
    }
  }
}

//////////////////////////////////////////////////
void PosePublisher::PostUpdate(const UpdateInfo &_info,
                               const EntityComponentManager &_ecm)
{
  GZ_PROFILE("PosePublisher::PostUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  bool publish = true;
  auto diff = _info.simTime - lastPosePubTime;
  // If the diff is positive and it's less than the update period, we skip
  // publication. If the diff is negative, then time has gone backward, we go
  // ahead publish and allow the time to be reset
  if ((diff > std::chrono::steady_clock::duration::zero()) &&
      (diff < updatePeriod))
  {
    publish = false;
  }

  bool publishStatic = true;
  auto staticDiff = _info.simTime - lastStaticPosePubTime;
  if (!staticPosePublisher ||
      ((staticDiff > std::chrono::steady_clock::duration::zero()) &&
      (staticDiff < staticUpdatePeriod)))
  {
    publishStatic = false;
  }

  if (!publish && !publishStatic)
    return;

  if (!initialized)
  {
    InitializeEntitiesToPublish(_ecm);
    initialized = true;
  }


  // if static transforms are published through a different topic
  if (staticPosePublisher)
  {
    if (publishStatic)
    {
      staticPoses.clear();
      FillPoses(_ecm, staticPoses, true);
      PublishPoses(staticPoses,
          convert<msgs::Time>(_info.simTime), poseStaticPub);
      lastStaticPosePubTime = _info.simTime;
    }

    if (publish)
    {
      poses.clear();
      FillPoses(_ecm, poses, false);
      PublishPoses(poses,
          convert<msgs::Time>(_info.simTime), posePub);
      lastPosePubTime = _info.simTime;
    }
  }
  // publish all transforms to the same topic
  else if (publish)
  {
    poses.clear();
    FillPoses(_ecm, poses, true);
    FillPoses(_ecm, poses, false);
    PublishPoses(poses,
        convert<msgs::Time>(_info.simTime), posePub);
    lastPosePubTime = _info.simTime;
  }
}

//////////////////////////////////////////////////
void PosePublisher::InitializeEntitiesToPublish(
    const EntityComponentManager &_ecm)
{
  std::stack<Entity> toCheck;
  toCheck.push(this->model.Entity());
  std::vector<Entity> visited;
  while (!toCheck.empty())
  {
    Entity entity = toCheck.top();
    toCheck.pop();
    visited.push_back(entity);

    auto link = _ecm.Component<components::Link>(entity);
    auto visual = _ecm.Component<components::Visual>(entity);
    auto collision = _ecm.Component<components::Collision>(entity);
    auto sensor = _ecm.Component<components::Sensor>(entity);
    auto joint = _ecm.Component<components::Joint>(entity);

    auto isModel = _ecm.Component<components::Model>(entity);
    auto parent = _ecm.Component<components::ParentEntity>(entity);

    bool fillPose = (link && this->publishLinkPose) ||
        (visual && this->publishVisualPose) ||
        (collision && this->publishCollisionPose) ||
        (sensor && this->publishSensorPose);

    // for backward compatibility, top level model pose will be published
    // if publishNestedModelPose is set to true unless the user explicity
    // disables this by setting publishModelPose to false
    if (isModel)
    {
      if (parent)
      {
        auto nestedModel = _ecm.Component<components::Model>(parent->Data());
        if (nestedModel)
          fillPose = this->publishNestedModelPose;
      }
      if (!fillPose)
      {
        fillPose = this->publishNestedModelPose && this->publishModelPose;
      }
    }

    if (fillPose)
    {
      std::string frame;
      std::string childFrame;
      auto entityName = _ecm.Component<components::Name>(entity);
      if (!entityName)
        continue;
      childFrame =
        removeParentScope(scopedName(entity, _ecm, "::", false), "::");

      if (parent)
      {
        auto parentName = _ecm.Component<components::Name>(parent->Data());
        if (parentName)
        {
          frame = removeParentScope(
              scopedName(parent->Data(), _ecm, "::", false), "::");
        }
      }
      this->entitiesToPublish[entity] = std::make_pair(frame, childFrame);
    }

    // get dynamic entities
    if (this->staticPosePublisher && joint)
    {
      sdf::JointType jointType =
          _ecm.Component<components::JointType>(entity)->Data();
      if (jointType != sdf::JointType::INVALID &&
          jointType != sdf::JointType::FIXED)
      {
        std::string parentLinkName =
            _ecm.Component<components::ParentLinkName>(entity)->Data();
        std::string childLinkName =
            _ecm.Component<components::ChildLinkName>(entity)->Data();

        auto parentLinkEntity = _ecm.EntityByComponents(
            components::Name(parentLinkName), components::Link(),
            components::ParentEntity(this->model.Entity()));
        auto childLinkEntity = _ecm.EntityByComponents(
            components::Name(childLinkName), components::Link(),
            components::ParentEntity(this->model.Entity()));

        // add to list if not a canonical link
        if (!_ecm.Component<components::CanonicalLink>(parentLinkEntity))
          this->dynamicEntities.insert(parentLinkEntity);
        if (!_ecm.Component<components::CanonicalLink>(childLinkEntity))
          this->dynamicEntities.insert(childLinkEntity);
      }
    }

    // Recursively check if child entities need to be published
    auto childEntities =
        _ecm.ChildrenByComponents(entity, components::ParentEntity(entity));

    // Use reverse iterators to match the order of entities found so as to match
    // the expected order in the pose_publisher integration test.
    for (auto childIt = childEntities.rbegin(); childIt != childEntities.rend();
         ++childIt)
    {
      auto it = std::find(visited.begin(), visited.end(), *childIt);
      if (it == visited.end())
      {
        // Only add to stack if the entity hasn't been already been visited.
        // This also ensures there are no cycles.
        toCheck.push(*childIt);
      }
    }
  }

  // sanity check to make sure dynamicEntities are a subset of entitiesToPublish
  for (auto const &ent : this->dynamicEntities)
  {
    if (this->entitiesToPublish.find(ent) == this->entitiesToPublish.end())
    {
      gzwarn << "Entity id: '" << ent << "' not found when creating a list "
              << "of dynamic entities in pose publisher." << std::endl;
    }
  }

  if (this->staticPosePublisher)
  {
    this->poses.reserve(this->dynamicEntities.size());
    this->staticPoses.reserve(
        this->entitiesToPublish.size() - this->dynamicEntities.size());
  }
  else
  {
    this->poses.reserve(this->entitiesToPublish.size());
  }
}

//////////////////////////////////////////////////
void PosePublisher::FillPoses(const EntityComponentManager &_ecm,
    std::vector<std::pair<Entity, math::Pose3d>> &_poses, bool _static)
{
  GZ_PROFILE("PosePublisher::FillPose");

  for (const auto &entity : this->entitiesToPublish)
  {
    auto pose = _ecm.Component<components::Pose>(entity.first);
    if (!pose)
      continue;

    bool isStatic = this->dynamicEntities.find(entity.first) ==
          this->dynamicEntities.end();

    if (_static == isStatic)
      _poses.emplace_back(entity.first, pose->Data());
  }
}

//////////////////////////////////////////////////
void PosePublisher::PublishPoses(
    std::vector<std::pair<Entity, math::Pose3d>> &_poses,
    const msgs::Time &_stampMsg,
    transport::Node::Publisher &_publisher)
{
  GZ_PROFILE("PosePublisher::PublishPoses");

  // publish poses
  msgs::Pose *msg = nullptr;
  if (this->usePoseV)
    this->poseVMsg.Clear();

  for (const auto &[entity, pose] : _poses)
  {
    auto entityIt = this->entitiesToPublish.find(entity);
    if (entityIt == this->entitiesToPublish.end())
      continue;

    if (this->usePoseV)
    {
      msg = this->poseVMsg.add_pose();
    }
    else
    {
      this->poseMsg.Clear();
      msg = &this->poseMsg;
    }

    // fill pose msg
    // frame_id: parent entity name
    // child_frame_id = entity name
    // pose is the transform from frame_id to child_frame_id
    GZ_ASSERT(msg != nullptr, "Pose msg is null");
    auto header = msg->mutable_header();

    header->mutable_stamp()->CopyFrom(_stampMsg);
    const std::string &frameId = entityIt->second.first;
    const std::string &childFrameId = entityIt->second.second;
    const math::Pose3d &transform = pose;
    auto frame = header->add_data();
    frame->set_key("frame_id");
    frame->add_value(frameId);
    auto childFrame = header->add_data();
    childFrame->set_key("child_frame_id");
    childFrame->add_value(childFrameId);

    // set pose
    msg->set_name(childFrameId);
    msgs::Set(msg, transform);

    // publish individual pose msgs
    if (!this->usePoseV)
      _publisher.Publish(this->poseMsg);
  }

  // publish pose vector msg
  if (this->usePoseV)
    _publisher.Publish(this->poseVMsg);
}

GZ_ADD_PLUGIN(PosePublisher,
              System,
              PosePublisher::ISystemConfigure,
              PosePublisher::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(PosePublisher,
                    "gz::sim::systems::PosePublisher")
