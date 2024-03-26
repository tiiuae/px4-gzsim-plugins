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
#include "gz/sim/Util.hh"

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

    auto world_entiry = gz::sim::worldEntity(_ecm);
    auto world_has_origin{false};

    // Use environment variables if set for home position.
    const char *env_lat = std::getenv("PX4_HOME_LAT");
    const char *env_lon = std::getenv("PX4_HOME_LON");
    const char *env_alt = std::getenv("PX4_HOME_ALT");

    if (env_lat) {
        lat_home_ = std::stod(env_lat) * M_PI / 180.0;
        gzmsg << "[gazebo_groundtruth_plugin] Home latitude is set to " << std::stod(env_lat) << ".\n";
    } else if (world_has_origin) {
        lat_home_ = world_latitude_;
        gzmsg << "[gazebo_groundtruth_plugin] Home latitude is set to " << lat_home_ << ".\n";
    } else if(_sdf->HasElement("homeLatitude")) {
        double latitude = _sdf->Get<double>("homeLatitude", lat_home_).first;
        lat_home_ = latitude * M_PI / 180.0;
    }

    if (env_lon) {
        lon_home_ = std::stod(env_lon) * M_PI / 180.0;
        gzmsg << "[gazebo_groundtruth_plugin] Home longitude is set to " << std::stod(env_lon) << ".\n";
    } else if (world_has_origin) {
        lon_home_ = world_longitude_;
        gzmsg << "[gazebo_groundtruth_plugin] Home longitude is set to " << lon_home_ << ".\n";
    } else if(_sdf->HasElement("homeLongitude")) {
        double longitude = _sdf->Get<double>("homeLongitude", lon_home_).first;
        lon_home_ = longitude * M_PI / 180.0;
    }

    if (env_alt) {
        alt_home_ = std::stod(env_alt);
        gzmsg << "[gazebo_groundtruth_plugin] Home altitude is set to " << alt_home_ << ".\n";
    } else if (world_has_origin) {
        alt_home_ = world_altitude_;
        gzmsg << "[gazebo_groundtruth_plugin] Home altitude is set to " << alt_home_ << ".\n";
    } else if(_sdf->HasElement("homeAltitude")) {
        alt_home_ = _sdf->Get<double>("homeAltitude", alt_home_).first;
    }

    posePub = node.Advertise<msgs::Pose>(poseTopic);
    navPub = node.Advertise<msgs::NavSat>(poseTopic + "/gps");
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
    msgs::NavSat *n_Msg{nullptr};

    auto pose = _ecm.Component<components::Pose>(model.Entity());
    if (!pose)
        return;

    this->poseMsg.Clear();
    msg = &this->poseMsg;
    navMsg.Clear();
    n_Msg = &navMsg;


    // fill pose msg
    // pose is the transform from parent_name_ to model_name_
    GZ_ASSERT(msg != nullptr, "Pose msg is null");
    GZ_ASSERT(n_Msg != nullptr, "Navigation msg is null");
    auto header = msg->mutable_header();

    auto n_header = n_Msg->mutable_header();

    header->mutable_stamp()->CopyFrom(_stampMsg);
    n_header->mutable_stamp()->CopyFrom(_stampMsg);
    const math::Pose3d &transform = pose->Data();
    auto childFrame = header->add_data();
    childFrame->set_key("model_name");
    childFrame->add_value(model_name_);

    auto pos_W_I = transform.Pos();
    auto att_W_I = transform.Rot();

    auto pos = sphericalCoordinates(model.Entity(), _ecm).value();

    // reproject position into geographic coordinates
    auto latlon_gt = reproject(pos_W_I, lat_home_, lon_home_, alt_home_);

//  TODO: Velocity

//    auto one = relativeVel(model.Entity(), _ecm);
//    gzwarn << "Velocity: " << one << std::endl;
//    gz::sim::worldPose(model.Entity());
//    gzwarn << "-Raw Pos(): " << transform.Pos() << std::endl;
//    gzwarn << "-Ready: " << latlon_gt.first << " " << latlon_gt.second << " " << pos_W_I.Z() + alt_home_ << " "
//           << att_W_I.W() << " " << att_W_I.X() << " " << att_W_I.Y() << " " << att_W_I.Z() << std::endl;
//    gzwarn << "-Converted: " << latlon_gt.first * 180 / M_PI << " " << latlon_gt.second * 180 / M_PI << std::endl;
    // set pose
    msg->set_name(model_name_);
    msgs::Set(msg, transform);

    n_Msg->set_latitude_deg(latlon_gt.first * 180 / M_PI);
    n_Msg->set_longitude_deg(latlon_gt.second * 180 / M_PI);
    n_Msg->set_altitude(pos_W_I.Z() + alt_home_);

    // publish individual pose msgs
    posePub.Publish(this->poseMsg);
    navPub.Publish(navMsg);
}

GZ_ADD_PLUGIN(GroundtruthPlugin,
              System,
              GroundtruthPlugin::ISystemConfigure,
              GroundtruthPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(GroundtruthPlugin,
                    "gz::sim::systems::GroundtruthPlugin")
