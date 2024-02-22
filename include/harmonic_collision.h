#pragma once

#include <memory>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <gz/sim/Model.hh>

#include "custom_message.pb.h"

namespace gz {
    using namespace sim;

    /// \brief Plugin which publishes a message if the model it is attached
    /// to has collision.
    ///
    ///
    /// The plugin requires that a contact sensors is placed in at least one
    /// link on the model on which this plugin is attached.
    class CollisionPlugin
            : public System,
              public ISystemConfigure,
              public ISystemPreUpdate,
              public ISystemPostUpdate {
    public:
        void Load(const EntityComponentManager &_ecm,
                  const sdf::ElementPtr &_sdf);

        /// \brief Process contact sensor data and determine if a touch event occurs
        /// \param[in] _info Simulation update info
        /// \param[in] _ecm Immutable reference to the EntityComponentManager
        void Update(const UpdateInfo &_info,
                    EntityComponentManager const &_ecm);

        CollisionPlugin() : System() {}

        ~CollisionPlugin() override = default;

        void Configure(const Entity &_entity,
                       const std::shared_ptr<const sdf::Element> &_sdf,
                       EntityComponentManager &_ecm,
                       EventManager &_eventMgr) override;

        void PreUpdate(const UpdateInfo &_info,
                       EntityComponentManager &_ecm) final;

        void PostUpdate(
                const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) final;


    private:
        std::string getObjName(std::string const &object);

        /// \brief Model interface
        Model model{kNullEntity};

        /// \brief Transport node to keep services alive
        transport::Node node;

        /// \brief Collision entities that have been designated as contact sensors.
        /// These will be checked against the targetEntities to establish whether this
        /// model is touching the targets
        std::vector<Entity> collisionEntities;

        /// \brief Publisher which publishes a message after touched for enough time
        transport::Node::Publisher publisher_;

        /// \brief Copy of the sdf configuration used for this plugin
        sdf::ElementPtr sdfConfig;

        /// \brief Initialization flag
        bool initialized{false};

        std::string my_name_{"unknown"};

        uint64_t last_time_{0};

        uint64_t current_time_{0};

        int32_t publish_rate_{500};

        std::set<std::string> objs_hit_{};
    };

// For debug
//    void subscriberCallback(const gz::msgs::CollisionObject &_msg);
}