
#ifndef VIBRATION_PLUGIN_HH_
#define VIBRATION_PLUGIN_HH_

#include <common.h>
#include <math.h>
#include <sdf/sdf.hh>

#include <random>

#include <gz/math/Vector3.hh>
#include <gz/msgs.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <math.h>

#include <ignition/math.hh>

static const std::string kDefaultVibrationPluginEnable = "/vibration/enable";

namespace vibration_plugin {

class VibrationPlugin : public gz::sim::System,
                        public gz::sim::ISystemConfigure,
                        public gz::sim::ISystemPreUpdate {

public:
  VibrationPlugin() = default;

  void Configure(const gz::sim::Entity& _entity, const std::shared_ptr<const sdf::Element>& _sdf,
                 gz::sim::EntityComponentManager& _ecm, gz::sim::EventManager& _eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm) override;

private:
  gz::sim::Entity model_entity_{gz::sim::kNullEntity};
  gz::sim::Entity link_entity_{gz::sim::kNullEntity};

  std::shared_ptr<gz::transport::Node> node_;
  // gz::transport::Node::Publisher pub_; // Add this

  std::atomic<bool> enabled_{false};

  double amplitude_{10.0};
  gz::math::Vector3d axis_{0, 0, 1};
  double frequency_{2.0};
  int last_sign{1};

  std::mt19937 random_device{std::random_device{}()};
  std::uniform_real_distribution<double> distribution{-1.0, 1.0};
};
}; // namespace vibration_plugin

#endif
