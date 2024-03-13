#include "groundtruth_plugin.h"

#include <gz/msgs/pose.pb.h>
#include <gz/msgs/time.pb.h>

#include <string>

#include <sdf/Joint.hh>

#include <gz/common/Profiler.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>

#include "gz/sim/Util.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Conversions.hh"

using namespace gz;
using namespace sim;
using namespace systems;

//////////////////////////////////////////////////
GroundtruthPlugin::GroundtruthPlugin() {
}

//////////////////////////////////////////////////
void GroundtruthPlugin::Configure(const Entity &_entity,
                                  const std::shared_ptr<const sdf::Element> &_sdf,
                                  EntityComponentManager &_ecm,
                                  EventManager &/*_eventMgr*/) {
    model = Model(_entity);

    if (!model.Valid(_ecm)) {
        gzerr << "PosePublisher plugin should be attached to a model entity. "
              << "Failed to initialize." << std::endl;
        return;
    }
    model_name_ = removeParentScope(scopedName(_entity, _ecm, "::", false), "::");

    double updateFrequency = _sdf->Get<double>("update_frequency", -1).first;

    if (updateFrequency > 0) {
        std::chrono::duration<double> period{1 / updateFrequency};
        updatePeriod = std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
    }

    // create publisher
    std::string poseTopic = scopedName(_entity, _ecm) + "/groundtruth";
    poseTopic = transport::TopicUtils::AsValidTopic(poseTopic);
    if (poseTopic.empty()) {
        poseTopic = "/groundtruth";
        gzerr << "Empty pose topic generated for pose_publisher system. "
              << "Setting to " << poseTopic << std::endl;
    }

    posePub = node.Advertise<msgs::Pose>(poseTopic);
}

//////////////////////////////////////////////////
void GroundtruthPlugin::PostUpdate(const UpdateInfo &_info,
                                   const EntityComponentManager &_ecm) {
    GZ_PROFILE("PosePublisher::PostUpdate");

    // \TODO(anyone) Support rewind
    if (_info.dt < std::chrono::steady_clock::duration::zero()) {
        gzwarn << "Detected jump back in time ["
               << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
               << "s]. System may not work properly." << std::endl;
    }

    // Nothing left to do if paused.
    if (_info.paused)
        return;


    auto diff = _info.simTime - lastPosePubTime;
    // If the diff is positive and it's less than the update period, we skip
    // publication. If the diff is negative, then time has gone backward, we go
    // ahead publish and allow the time to be reset
    if ((diff > std::chrono::steady_clock::duration::zero()) &&
        (diff < updatePeriod)) {
        return;
    }

    PublishPose(_ecm, convert<msgs::Time>(_info.simTime));
    lastPosePubTime = _info.simTime;
}

//////////////////////////////////////////////////
void GroundtruthPlugin::PublishPose(const EntityComponentManager &_ecm,
                                    const msgs::Time &_stampMsg) {
    GZ_PROFILE("PosePublisher::PublishPoses");

    // publish poses
    msgs::Pose *msg{nullptr};
    auto pose = _ecm.Component<components::Pose>(model.Entity());
    if (!pose)
        return;

    this->poseMsg.Clear();
    msg = &this->poseMsg;

    // fill pose msg
    // pose is the transform from parent_name_ to model_name_
    GZ_ASSERT(msg != nullptr, "Pose msg is null");
    auto header = msg->mutable_header();

    header->mutable_stamp()->CopyFrom(_stampMsg);
    const math::Pose3d &transform = pose->Data();
    auto childFrame = header->add_data();
    childFrame->set_key("model_name");
    childFrame->add_value(model_name_);

    // set pose
    msg->set_name(model_name_);
    msgs::Set(msg, transform);

    // publish individual pose msgs
    posePub.Publish(this->poseMsg);
}

GZ_ADD_PLUGIN(GroundtruthPlugin,
              System,
              GroundtruthPlugin::ISystemConfigure,
              GroundtruthPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(GroundtruthPlugin,
                    "gz::sim::systems::GroundtruthPlugin")
