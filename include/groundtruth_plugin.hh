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
#ifndef GZ_SIM_SYSTEMS_POSEPUBLISHER_HH_
#define GZ_SIM_SYSTEMS_POSEPUBLISHER_HH_

#include <memory>
#include <gz/sim/config.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include "gz/sim/Model.hh"
#include <gz/msgs/pose_v.pb.h>

namespace gz {
    namespace sim {
// Inline bracket to help doxygen filtering.
        inline namespace GZ_SIM_VERSION_NAMESPACE {
            namespace systems {

                /// \brief Pose publisher system. Attach to an entity to publish the
                /// transform of its child entities in the form of gz::msgs::Pose
                /// messages, or a single gz::msgs::Pose_V message if
                /// "use_pose_vector_msg" is true.
                ///
                /// ## System Parameters
                ///
                /// - `<publish_link_pose>`: Set to true to publish link pose
                /// - `<publish_visual_pose>`: Set to true to publish visual pose
                /// - `<publish_collision_pose>`: Set to true to publish collision pose
                /// - `<publish_sensor_pose>`: Set to true to publish sensor pose
                /// - `<publish_model_pose>`: Set to true to publish model pose.
                /// - `<publish_nested_model_pose>`: Set to true to publish nested model
                ///   pose. The pose of the model that contains this system is also published
                ///   unless publish_model_pose is set to false
                /// - `<use_pose_vector_msg>`: Set to true to publish a gz::msgs::Pose_V
                ///   message instead of mulitple gz::msgs::Pose messages.
                /// - `<update_frequency>`: Frequency of pose publications in Hz. A negative
                ///   frequency publishes as fast as possible (i.e, at the rate of the
                ///   simulation step)
                /// - `<static_publisher>`: Set to true to publish static poses on a
                ///   "<scoped_entity_name>/pose_static" topic. This will cause only dynamic
                ///   poses to be published on the "<scoped_entity_name>/pose" topic.
                /// - `<static_update_frequency>`: Frequency of static pose publications in
                ///   Hz. A negative frequency publishes as fast as possible (i.e, at the
                ///   rate of the simulation step).
                class PosePublisher
                        : public System,
                          public ISystemConfigure,
                          public ISystemPostUpdate {
                public:
                    /// \brief Constructor
                    PosePublisher();

                    /// \brief Destructor
                    ~PosePublisher() override = default;

                    // Documentation inherited
                    void Configure(const Entity &_entity,
                                   const std::shared_ptr<const sdf::Element> &_sdf,
                                   EntityComponentManager &_ecm,
                                   EventManager &_eventMgr) override;

                    // Documentation inherited
                    void PostUpdate(
                            const UpdateInfo &_info,
                            const EntityComponentManager &_ecm) override;

                    /// \brief Private data pointer
//    private: std::unique_ptr<PosePublisherPrivate> dataPtr;

                    /// \brief Initializes internal caches for entities whose poses are to be
                    /// published and their names
                    /// \param[in] _ecm Immutable reference to the entity component manager
                    void InitializeEntitiesToPublish(const EntityComponentManager &_ecm);

                    /// \brief Helper function to collect entity pose data
                    /// \param[in] _ecm Immutable reference to the entity component manager
                    /// \param[out] _poses Pose vector to be filled
                    /// \param[in] _static True to fill only static transforms,
                    /// false to fill only dynamic transforms
                    void FillPoses(const EntityComponentManager &_ecm,
                                   std::vector<std::pair<Entity, math::Pose3d>> &_poses,
                                   bool _static);

                    /// \brief Publishes poses collected by FillPoses with the provided time
                    /// stamp.
                    /// \param[in] _poses Pose to publish
                    /// \param[in] _stampMsg Time stamp associated with published poses
                    /// \param[in] _publisher Publisher to publish the message
                    void PublishPoses(
                            std::vector<std::pair<Entity, math::Pose3d>> &_poses,
                            const msgs::Time &_stampMsg,
                            transport::Node::Publisher &_publisher);

                private:
                    /// \brief Gazebo communication node.
                    transport::Node node;

                    /// \brief publisher for pose data
                    transport::Node::Publisher posePub;

                    /// \brief True to publish static transforms to a separate topic
                    bool staticPosePublisher = false;

                    /// \brief publisher for pose data
                    transport::Node::Publisher poseStaticPub;

                    /// \brief Model interface
                    Model model{kNullEntity};

                    /// \brief True to publish link pose
                    bool publishLinkPose = true;

                    /// \brief True to publish visual pose
                    bool publishVisualPose = false;

                    /// \brief True to publish collision pose
                    bool publishCollisionPose = false;

                    /// \brief True to publish sensor pose
                    bool publishSensorPose = false;

                    /// \brief True to publish nested model pose
                    bool publishNestedModelPose = false;

                    /// \brief True to publish model pose
                    bool publishModelPose = false;

                    /// \brief Frequency of pose publications in Hz. A negative frequency
                    /// publishes as fast as possible (i.e, at the rate of the simulation step)
                    double updateFrequency = -1;

                    /// \brief Last time poses were published.
                    std::chrono::steady_clock::duration lastPosePubTime{0};

                    /// \brief Last time static poses were published.
                    std::chrono::steady_clock::duration lastStaticPosePubTime{0};

                    /// \brief Update period in nanoseconds calculated from the update_frequency
                    /// parameter
                    std::chrono::steady_clock::duration updatePeriod{0};

                    /// \brief Update period in nanoseconds calculated from the
                    /// static_update_frequency parameter
                    std::chrono::steady_clock::duration staticUpdatePeriod{0};

                    /// \brief Cache of entities, their frame names and their child frame names.
                    /// The key is the entity whose pose is to be published.
                    /// The frame name is the scoped name of the parent entity.
                    /// The child frame name is the scoped name of the entity (the key)
                    std::unordered_map<Entity, std::pair<std::string, std::string>>
                            entitiesToPublish;

                    /// \brief Entities with pose that can change over time, i.e. links connected
                    /// by joints
                    std::unordered_set<Entity> dynamicEntities;

                    /// \brief A vector that contains the entities and their poses. This could
                    /// easily be a temporary, but having it as a member variable improves
                    /// performance by avoiding memory allocation
                    std::vector<std::pair<Entity, math::Pose3d>> poses;

                    /// \brief A vector that contains the entities and poses that are static.
                    /// This could easily be a temporary, but having it as a member variable
                    /// improves performance by avoiding memory allocation
                    std::vector<std::pair<Entity, math::Pose3d>> staticPoses;

                    /// \brief A variable that gets populated with poses. This also here as a
                    /// member variable to avoid repeated memory allocations and improve
                    /// performance.
                    msgs::Pose poseMsg;

                    /// \brief A variable that gets populated with poses. This also here as a
                    /// member variable to avoid repeated memory allocations and improve
                    /// performance.
                    msgs::Pose_V poseVMsg;

                    /// \brief True to publish a vector of poses. False to publish individual pose
                    /// msgs.
                    bool usePoseV = false;

                    /// \brief Whether cache variables have been initialized
                    bool initialized{false};
                };
            }
        }
    }
}

#endif
