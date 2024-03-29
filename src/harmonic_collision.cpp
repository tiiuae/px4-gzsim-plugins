
#include "harmonic_collision.h"

#include <gz/plugin/Register.hh>
#include <gz/sim/components/ContactSensor.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/Util.hh>

using namespace gz;
using namespace sim;
using namespace systems;

void CollisionPlugin::Load(const EntityComponentManager &_ecm,
                           const sdf::ElementPtr &_sdf) {
    // Create a list of collision entities that have been marked as contact
    // sensors in this model. These are collisions that have a ContactSensorData
    // component
    auto allLinks =
            _ecm.ChildrenByComponents(this->model.Entity(), components::Link());

    for (const Entity linkEntity: allLinks) {
        auto linkCollisions =
                _ecm.ChildrenByComponents(linkEntity, components::Collision());
        for (const Entity colEntity: linkCollisions) {
            if (_ecm.EntityHasComponentType(colEntity,
                                            components::ContactSensorData::typeId)) {
                this->collisionEntities.push_back(colEntity);
            }
        }
    }

    if (_sdf->HasElement("updateRate")) {
        publish_rate_ = _sdf->GetElement("updateRate")->Get<int32_t>();
        gzmsg << "Publish rate set to " << publish_rate_ << std::endl;
    }

    my_name_ = model.Name(_ecm);
    auto topic = my_name_ + "/collisions";
    publisher_ = node.Advertise<gz::msgs::CollisionObject>(topic);

// For debug
//    node.Subscribe(topic, subscriberCallback);
}

void CollisionPlugin::Update(UpdateInfo const &_info,
                             EntityComponentManager const &_ecm) {
    // Iterate through all the target entities and check if there is a contact
    // between the target entity and this model
    for (const Entity colEntity: this->collisionEntities) {
        auto *contacts = _ecm.Component<components::ContactSensorData>(colEntity);
        if (contacts) {
            for (const auto &contact: contacts->Data().contact()) {

                std::tuple<std::string, std::string> objects = {getObjName(contact.collision1().name()),
                                                                getObjName(contact.collision2().name())};

                if (auto first_obj = std::get<0>(objects), second_obj = std::get<1>(objects); first_obj.empty())
                    objs_hit_.insert(second_obj);
                else
                    objs_hit_.insert(first_obj);
            }
        }
    }

    current_time_ = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();

    if (current_time_ - last_time_ > publish_rate_) {
        gz::msgs::CollisionObject msg;
        static int32_t sequence_id_{0};
        msg.set_sequence_id(++sequence_id_);
        msg.set_device_name(my_name_);

        if (objs_hit_.empty())
            msg.set_object("");
        else {
            std::string objects;

            for (auto const &obj: objs_hit_)
                objects += obj + " ";

            msg.set_object(objects);
        }
        publisher_.Publish(msg);

        if (sequence_id_ >= INT32_MAX)
            sequence_id_ = 0;

        last_time_ = current_time_;
        objs_hit_.clear();
    }
}

void CollisionPlugin::Configure(const Entity &_entity,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                EntityComponentManager &_ecm, EventManager &) {
    model = Model(_entity);
    if (!model.Valid(_ecm)) {
        gzerr << "Touch plugin should be attached to a model entity. "
              << "Failed to initialize." << std::endl;
        return;
    }
    sdfConfig = _sdf->Clone();
}

void CollisionPlugin::PreUpdate(const UpdateInfo &, EntityComponentManager &_ecm) {
    if ((!initialized) && sdfConfig) {
        // We call Load here instead of Configure because we can't be guaranteed
        // that all entities have been created when Configure is called
        Load(_ecm, sdfConfig);
        initialized = true;
    }
}

void CollisionPlugin::PostUpdate(const UpdateInfo &_info,
                                 const EntityComponentManager &_ecm) {
    Update(_info, _ecm);
}

std::string CollisionPlugin::getObjName(std::string const &object) {
    std::vector<std::string> uninteresting{"ground", "sun", my_name_};
    for (auto const &obj: uninteresting)
        if (object.find(obj) != std::string::npos)
            return {};

    return {object.substr(0, object.find("::"))};
}

// For debug
//void gz::subscriberCallback(const gz::msgs::CollisionObject &_msg) {
//    gzwarn << __FUNCTION__ << ": " << _msg.DebugString() << std::endl;
//}

GZ_ADD_PLUGIN(CollisionPlugin,
              System,
              CollisionPlugin::ISystemConfigure,
              CollisionPlugin::ISystemPreUpdate,
              CollisionPlugin::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(CollisionPlugin, "gz::CollisionPlugin")