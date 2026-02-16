#include <gazebo_vibration_plugin.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <random>

namespace vibration_plugin {

void VibrationPlugin::Configure(const gz::sim::Entity& _entity,
                                const std::shared_ptr<const sdf::Element>& _sdf,
                                gz::sim::EntityComponentManager& _ecm,
                                gz::sim::EventManager& _eventMgr) {

  // Initialize topic name from SDF or use default

  gz::transport::NodeOptions opts;
  opts.SetPartition("sim");
  this->node_ = std::make_shared<gz::transport::Node>(opts);

  // Subscribe with explicit callback
  auto callback =
      std::function<void(const gz::msgs::Boolean&)>([this](const gz::msgs::Boolean& msg) {
        std::cout << "[VibrationPlugin] Vibration enabled" << msg.data() << std::endl;

        this->enabled_.store(msg.data());
      });

  bool subscribed = this->node_->Subscribe(kDefaultVibrationPluginEnable, callback);
  if (!subscribed) {
    std::cerr << "[VibrationPlugin] Failed to subscribe to topic: " << kDefaultVibrationPluginEnable
              << std::endl;
    return;
  }

  // Wait a bit for transport layer to settle
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // this->pub_ = this->node_->Advertise<gz::msgs::Boolean>(kDefaultVibrationPluginEnable);

  // List all topics to verify transport is working
  std::ostringstream oss;
  std::vector<std::string> topics;
  this->node_->TopicList(topics);
  std::cout << "[VibrationPlugin] Available topics count: " << topics.size() << std::endl;
  oss << "[VibrationPlugin] Topics list:\n";
  for (const auto& topic : topics) {
    oss << "  - " << topic << "\n";
  }
  std::cout << oss.str() << std::flush;
  std::cout << "[VibrationPlugin] Subscribed to: " << kDefaultVibrationPluginEnable << std::endl;
  std::cout << "[VibrationPlugin] Configuration complete:" << std::endl;
  std::cout << "  Topic: " << kDefaultVibrationPluginEnable << std::endl;
  std::cout << "  Link: " << "" << std::endl;
  std::cout << "  Amplitude: " << this->amplitude_ << " N" << std::endl;
  std::cout << "  Frequency: " << this->frequency_ << " Hz" << std::endl;
  std::cout << "  Axis: [" << this->axis_.X() << ", " << this->axis_.Y() << ", " << this->axis_.Z()
            << "]" << std::endl;

  std::cout << "[VibrationPlugin] Subscribed to: " << kDefaultVibrationPluginEnable << std::endl;

  this->model_entity_ = _entity;
  gz::sim::Model model(_entity);

  if (!model.Valid(_ecm))
    return;

  std::string link_name = "base_link";

  if (_sdf->HasElement("link_name"))
    link_name = _sdf->Get<std::string>("link_name");

  this->link_entity_ = model.LinkByName(_ecm, link_name);

  if (this->link_entity_ == gz::sim::kNullEntity) {
    std::cerr << "[VibrationPlugin] Failed to find link: " << link_name << std::endl;
    return;
  }

  if (_sdf->HasElement("amplitude"))
    this->amplitude_ = _sdf->Get<double>("amplitude");

  if (_sdf->HasElement("frequency"))
    this->frequency_ = _sdf->Get<double>("frequency");

  if (_sdf->HasElement("axis"))
    this->axis_ = _sdf->Get<gz::math::Vector3d>("axis");

  // Normalize the axis vector
  this->axis_.Normalize();

  std::cout << "[VibrationPlugin] Configuration complete:" << std::endl;
  std::cout << "  Topic: " << kDefaultVibrationPluginEnable << std::endl;
  std::cout << "  Link: " << link_name << std::endl;
  std::cout << "  Amplitude: " << this->amplitude_ << " N" << std::endl;
  std::cout << "  Frequency: " << this->frequency_ << " Hz" << std::endl;
  std::cout << "  Axis: [" << this->axis_.X() << ", " << this->axis_.Y() << ", " << this->axis_.Z()
            << "]" << std::endl;
};

void VibrationPlugin::PreUpdate(const gz::sim::UpdateInfo& _info,
                                gz::sim::EntityComponentManager& _ecm) {

  // if not enable by the topic then dont apply the vibration
  if (!this->enabled_)
    return;

  if (_info.paused)
    return;

  double time = std::chrono::duration<double>(_info.simTime).count();

  // double applied_force_magnitude =
  //     this->amplitude_ * sin(2.0 * M_PI * this->frequency_ * time) + this->amplitude_;

  double applied_force_magnitude = distribution(random_device) * this->amplitude_;

  int current_sign = 1;
  std::copysign(current_sign, applied_force_magnitude);

  if (current_sign == last_sign) {
    last_sign = -1 * current_sign;
  }

  this->last_sign = current_sign;

  gz::math::Vector3d force = this->last_sign * this->axis_ * applied_force_magnitude;

  gz::math::Vector3d torque = this->last_sign * this->axis_ * applied_force_magnitude;

  // Get or create wrench component
  auto wrenchComp = _ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(this->link_entity_);

  if (!wrenchComp) {
    _ecm.CreateComponent(this->link_entity_,
                         gz::sim::components::ExternalWorldWrenchCmd(gz::msgs::Wrench()));
    wrenchComp = _ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(this->link_entity_);
  }

  gz::msgs::Wrench wrenchMsg;

  wrenchMsg.mutable_force()->set_x(force.X());
  wrenchMsg.mutable_force()->set_y(force.Y());
  wrenchMsg.mutable_force()->set_z(0);

  wrenchMsg.mutable_torque()->set_x(torque.X());
  wrenchMsg.mutable_torque()->set_y(torque.Y());
  wrenchMsg.mutable_torque()->set_z(torque.Z());

  wrenchComp->SetData(
      wrenchMsg, [](const gz::msgs::Wrench&, const gz::msgs::Wrench&) -> bool { return true; });
}
}; // namespace vibration_plugin

GZ_ADD_PLUGIN(vibration_plugin::VibrationPlugin, gz::sim::System,
              vibration_plugin::VibrationPlugin::ISystemConfigure,
              vibration_plugin::VibrationPlugin::ISystemPreUpdate);

GZ_ADD_PLUGIN_ALIAS(vibration_plugin::VibrationPlugin, "custom::VibrationPlugin");