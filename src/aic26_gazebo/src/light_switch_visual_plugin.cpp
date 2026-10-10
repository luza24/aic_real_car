#include <mutex>
#include <string>
#include <functional>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Light.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/rendering.hh>

#include <ros/ros.h>

namespace {
// "" until the parameter holds a valid mode
std::string mode_param(const ros::NodeHandle& nh) {
    std::string mode;
    if (!nh.getParam("day_night", mode) && !ros::param::get("/day_night", mode)) {
        return "";
    }
    return (mode == "day" || mode == "night") ? mode : "";
}

// Lights and visuals in the client scene carry fully scoped names, e.g.
// "armbot::base_footprint::left_headlight_light". Matching on a substring
// keeps this working when links get lumped together by fixed joints.
bool name_has(const std::string& _name, const std::string& _needle) {
    return _name.find(_needle) != std::string::npos;
}
}  // namespace

namespace gazebo {

// Switches the scene between day and night from inside the gzclient process.
//
// The sun and the URDF headlights are rendering-only entities: they only exist
// in the client scene. A WorldPlugin or ModelPlugin runs in gzserver, where
// rendering::get_scene() always returns null, so the switching has to happen
// here in a VisualPlugin. It is attached to the floor visual of the world,
// which is always present, and then drives every light/visual by name.
class LightSwitchVisualPlugin : public VisualPlugin {
public:
    void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf) override {
        (void)_visual;
        (void)_sdf;

        // no ROS call here: Load can run while gazebo holds its factory lock
        this->conn_ = gazebo::event::Events::ConnectPreRender(
            std::bind(&LightSwitchVisualPlugin::OnPreRender, this));
    }

private:
    void InitRos() {
        static std::mutex init_mutex;
        std::lock_guard<std::mutex> lock(init_mutex);
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "light_switch", ros::init_options::NoSigintHandler);
        }
        this->nh_.reset(new ros::NodeHandle("~"));
    }

    void OnPreRender() {
        if (!this->nh_) this->InitRos();
        if (!this->nh_) return;

        const std::string mode = mode_param(*this->nh_);
        if (mode.empty() || (this->applied_ && mode == this->mode_)) return;

        rendering::ScenePtr scene = rendering::get_scene();
        if (!scene) return;

        // headlights may not be spawned yet; wait until they show up
        std::vector<rendering::LightPtr> headlights;
        for (uint32_t i = 0; i < scene->LightCount(); ++i) {
            rendering::LightPtr light = scene->LightByIndex(i);
            if (light && name_has(light->Name(), "headlight")) {
                headlights.push_back(light);
            }
        }
        if (headlights.empty()) return;

        const bool day = (mode == "day");

        // sun: on during the day, off at night
        rendering::LightPtr sun = scene->LightByName("sun");
        if (sun) {
            sun->SetVisible(day);
            sun->SetDiffuseColor(day ? ignition::math::Color(0.9, 0.9, 0.9)
                                     : ignition::math::Color(0.0, 0.0, 0.0));
        }

        // headlights: on at night, off during the day
        for (const rendering::LightPtr& light : headlights) {
            light->SetVisible(!day);
        }

        // bulbs glow at night
        rendering::VisualPtr root = scene->WorldVisual();
        this->SetBulbEmissive(root, day);

        this->mode_ = mode;
        this->applied_ = true;
        ROS_INFO("[light_switch] '%s': sun=%s headlights=%u", mode.c_str(),
                 day ? "on" : "off", static_cast<unsigned>(headlights.size()));
    }

    void SetBulbEmissive(rendering::VisualPtr _visual, bool _day) {
        if (!_visual) return;

        // "::" separates the path segments, so a headlight bulb visual is the
        // leaf whose name contains "headlight"
        const std::string name = _visual->Name();
        if (name_has(name, "headlight") &&
            name.find("headlight", name.find_last_of(':')) != std::string::npos) {
            _visual->SetEmissive(_day ? ignition::math::Color(0.0, 0.0, 0.0)
                                      : ignition::math::Color(1.0, 1.0, 1.0));
        }

        for (unsigned int i = 0; i < _visual->GetChildCount(); ++i) {
            this->SetBulbEmissive(_visual->GetChild(i), _day);
        }
    }

    std::string mode_;
    bool applied_ = false;
    std::unique_ptr<ros::NodeHandle> nh_;
    gazebo::event::ConnectionPtr conn_;
};

GZ_REGISTER_VISUAL_PLUGIN(LightSwitchVisualPlugin)

}  // namespace gazebo
