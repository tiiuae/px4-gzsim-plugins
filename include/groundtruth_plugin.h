#pragma once

#include <memory>
#include <gz/sim/config.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include "gz/sim/Model.hh"
#include "common.h"
#include "Groundtruth.pb.h"

namespace gz {
    namespace sim {
// Inline bracket to help doxygen filtering.
        inline namespace GZ_SIM_VERSION_NAMESPACE {
            namespace systems {

                /// \brief Pose publisher system. Attach to an entity to publish the
                /// transform of its child entities in the form of gz::msgs::Pose
                /// messages
                ///
                /// ## System Parameters
                ///
                /// - `<update_frequency>`: Frequency of pose publications in Hz. A negative
                ///   frequency publishes as fast as possible (i.e, at the rate of the
                ///   simulation step)
                class GroundtruthPlugin
                        : public System,
                          public ISystemConfigure,
                          public ISystemPostUpdate {
                public:
                    /// \brief Constructor
                    GroundtruthPlugin();

                    /// \brief Destructor
                    ~GroundtruthPlugin() override = default;

                    // Documentation inherited
                    void Configure(const Entity &_entity,
                                   const std::shared_ptr<const sdf::Element> &_sdf,
                                   EntityComponentManager &_ecm,
                                   EventManager &_eventMgr) override;

                    // Documentation inherited
                    void PostUpdate(
                            const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) override;

                    /// \brief Publishes pose with the provided time
                    /// stamp.
                    /// \param[in] _stampMsg Time stamp associated with published poses
                    void PublishPose(const EntityComponentManager &_ecm,
                                     const msgs::Time &_stampMsg);

                private:
                    /// \brief Gazebo communication node.
                    transport::Node node;

                    /// \brief publisher for groundtruth data
                    transport::Node::Publisher navPub;

                    /// \brief Model interface
                    Model model{kNullEntity};

                    /// \brief Frequency of pose publications in Hz. A negative frequency
                    /// publishes as fast as possible (i.e, at the rate of the simulation step)
                    double updateFrequency = -1;

                    /// \brief Last time poses were published.
                    std::chrono::steady_clock::duration lastPosePubTime{0};

                    /// \brief Update period in nanoseconds calculated from the update_frequency
                    /// parameter
                    std::chrono::steady_clock::duration updatePeriod{0};

                    /// \brief Model name
                    std::string model_name_{};

                    // Home defaults to Zurich Irchel Park
                    // @note The home position can be specified using the environment variables:
                    // PX4_HOME_LAT, PX4_HOME_LON, and PX4_HOME_ALT
                    double lat_home_ = kDefaultHomeLatitude;
                    double lon_home_ = kDefaultHomeLongitude;
                    double alt_home_ = kDefaultHomeAltitude;
                    double world_latitude_ = 0.0;
                    double world_longitude_ = 0.0;
                    double world_altitude_ = 0.0;
                };
            }
        }
    }
}

#ifdef DEBUG
void callback(gz::msgs::Groundtruth const& msg);
#endif