#include "groundtruth_plugin.h"

#include <gz/msgs/time.pb.h>

#include <string>

#include <sdf/Joint.hh>

#include <gz/common/Profiler.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>

#include "gz/sim/Util.hh"
#include "gz/sim/components/ParentEntity.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/Conversions.hh"
// Just in case for Advertise function
// #include <gz/transport/AdvertiseOptions.hh>

using namespace gz;
using namespace sim;
using namespace systems;

//////////////////////////////////////////////////
GroundtruthPlugin::GroundtruthPlugin() {
}

namespace stash {
/// \brief Service callback can't use any of class fields (pub/priv/prot). It can use only global variables.
    static msgs::Groundtruth glob_gt_msg;
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
    std::string poseTopic = model_name_ + "/groundtruth";
    poseTopic = transport::TopicUtils::AsValidTopic(poseTopic);
    if (poseTopic.empty()) {
        poseTopic = "/groundtruth";
        gzerr << "Empty pose topic generated for pose_publisher system. "
              << "Setting to " << poseTopic << std::endl;
    }

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

    navPub = node.Advertise<msgs::Groundtruth>(poseTopic);
    auto resp = node.Advertise(poseTopic + "_req", &GroundtruthPlugin::responseCallback, {});
    if (!resp)
    {
        gzwarn << "Error advertising service" << std::endl;
    }
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

    fillPose(_ecm, convert<msgs::Time>(_info.simTime));
    // publish individual pose msgs
    navPub.Publish(gtMsg);
    lastPosePubTime = _info.simTime;
}

//////////////////////////////////////////////////
void GroundtruthPlugin::fillPose(const EntityComponentManager &_ecm,
                                 const msgs::Time &_stampMsg) {
    GZ_PROFILE("PosePublisher::PublishPoses");

    // publish poses
    auto pose = _ecm.Component<components::Pose>(model.Entity());
    if (!pose)
        return;

    // fill pose msg
    // pose is the transform from parent_name_ to model_name_
    const math::Pose3d &transform = pose->Data();
    auto pos_W_I = transform.Pos();
    auto att_W_I = transform.Rot();

    // reproject position into geographic coordinates
    auto latlon_gt = reproject(pos_W_I, lat_home_, lon_home_, alt_home_);

//  TODO: Velocity
//    auto one = relativeVel(model.Entity(), _ecm);
//    gzwarn << "Velocity: " << one << std::endl;

    // set pose
    static uint32_t i{0};
    msgs::Groundtruth *_gtMsg = &gtMsg;
    _gtMsg->Clear();
    _gtMsg->set_seq_num(i++);
    _gtMsg->set_time_sec(_stampMsg.sec());
    _gtMsg->set_time_nsec(_stampMsg.nsec());
    _gtMsg->set_latitude_rad(latlon_gt.first);
    _gtMsg->set_longitude_rad(latlon_gt.second);
    _gtMsg->set_altitude_m(pos_W_I.Z() + alt_home_);
    _gtMsg->set_velocity_east(0);
    _gtMsg->set_velocity_north(0);
    _gtMsg->set_velocity_up(0);
    _gtMsg->set_attitude_q_w(att_W_I.W());
    _gtMsg->set_attitude_q_x(att_W_I.X());
    _gtMsg->set_attitude_q_y(att_W_I.X());
    _gtMsg->set_attitude_q_z(att_W_I.Z());
    _gtMsg->set_frame_id(model_name_);

    stash::glob_gt_msg.CopyFrom(gtMsg);
}

GZ_ADD_PLUGIN(GroundtruthPlugin,
              System,
              GroundtruthPlugin::ISystemConfigure,
              GroundtruthPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(GroundtruthPlugin,
                    "gz::GroundtruthPlugin")

bool GroundtruthPlugin::responseCallback(const gz::msgs::StringMsg &, gz::msgs::Groundtruth &rep)
{
    rep.Clear();
    rep.CopyFrom(stash::glob_gt_msg);

    return true;
}