#include <mutex>
#include <string>
#include <cmath>
#include <limits>
#include <functional>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/Light.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/sky.pb.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace {
// sunrise/sunset defaults (hours) match the world <sky> element; overridable
// through /sky_sunrise and /sky_sunset (the day_night GUI edits those)
constexpr double kSunriseDefault = 6.0;
constexpr double kSunsetDefault = 20.0;

// how many extra frames a sky message keeps being re-sent after a change; the
// topic is not latched so a few retries cover a late subscriber. The retries
// are rate limited (see kSkyMinGap) because SkyX rebuilds its textures on every
// message and updating it too fast trips an OGRE buffer lock assertion.
constexpr int kSkyBurst = 3;
constexpr double kSkyMinGap = 0.5;  // seconds between sky messages

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

// time of day in hours [0..24) as a double, NaN when unset
double time_param(const ros::NodeHandle& nh) {
    double value = 0.0;
    if (nh.getParam("time", value) || ros::param::get("/sky_time", value)) {
        return value;
    }
    return std::numeric_limits<double>::quiet_NaN();
}

// the shorthand /day_night maps to a representative hour
double mode_to_time(const std::string& _mode) {
    return (_mode == "day") ? 12.0 : 0.0;
}

double param_or(const std::string& _name, double _fallback) {
    double value = 0.0;
    if (ros::param::get(_name, value)) {
        return value;
    }
    return _fallback;
}

double sunrise() { return param_or("/sky_sunrise", kSunriseDefault); }
double sunset() { return param_or("/sky_sunset", kSunsetDefault); }

bool is_day(double _time) {
    return _time >= sunrise() && _time < sunset();
}
}  // namespace

namespace gazebo {

// Drives the world sky box and the lights from ROS parameters, all from inside
// the gzclient process.
//
// The sky (SkyX) and the URDF headlights are rendering-only: they only exist in
// the client scene, and rendering::get_scene() is always null in gzserver. A
// WorldPlugin/ModelPlugin therefore cannot touch them. This VisualPlugin is
// attached to the floor visual of the world (always present) and does two
// things:
//   * reads /sky_time (hours) and republishes it on the "~/sky" transport topic
//     so SkyX moves its sun/moon and clouds, and
//   * derives day/night from that time (sunrise/sunset) and switches the world
//     "sun" light plus the robot headlights accordingly.
// The legacy /day_night parameter still works as a shorthand.
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
        if (this->nh_->hasParam("publish")) {
            this->publish_ = true;
        } else {
            ros::param::get("/sky_time_publish", this->publish_);
        }
        this->state_pub_ = this->nh_->advertise<std_msgs::Float64>("state", 1, true);
    }

    // the Scene listens on /gazebo/<world>/sky for sky updates
    void InitSkyTransport(const std::string& _world) {
        this->node_.reset(new transport::Node());
        this->node_->Init();
        const std::string topic = "/gazebo/" + _world + "/sky";
        this->sky_pub_ = this->node_->Advertise<msgs::Sky>(topic);
    }

    void OnPreRender() {
        if (!this->nh_) this->InitRos();
        if (!this->nh_) return;

        rendering::ScenePtr scene = rendering::get_scene();
        if (!scene) return;
        if (!this->node_) {
            // SkyX clouds render to an off-screen texture every frame and
            // updating them rapidly trips an OGRE buffer lock assertion on some
            // drivers, so keep the sky dome and moon but drop the clouds.
            scene->SetSkyXMode(rendering::Scene::GZ_SKYX_MOON);
            this->InitSkyTransport(scene->Name());
        }

        // resolve the requested time: /sky_time wins, /day_night is a shorthand
        double time = time_param(*this->nh_);
        if (std::isnan(time)) {
            const std::string mode = mode_param(*this->nh_);
            if (!mode.empty()) time = mode_to_time(mode);
        }
        if (std::isnan(time)) return;

        // normalise to [0..24)
        time = std::fmod(time, 24.0);
        if (time < 0.0) time += 24.0;

        const bool day = is_day(time);
        const bool time_changed = (std::abs(time - this->time_) > 1e-6);
        const bool first = !this->applied_;

        // Publish the sky time on change and for a short burst afterwards.
        // The very first messages can be dropped because the Scene may not have
        // subscribed to the sky topic yet, and the topic is not latched. The
        // retries are rate limited: SkyX rebuilds the sun/moon/atmosphere
        // textures on every message and pushing them too fast trips an OGRE
        // "Cannot lock this buffer" assertion and kills gzclient.
        if (time_changed || first) {
            this->sky_burst_ = kSkyBurst;
        }
        if (this->sky_burst_ > 0 &&
            (ros::WallTime::now() - this->last_sky_).toSec() >= kSkyMinGap) {
            this->PublishSky(time);
            this->last_sky_ = ros::WallTime::now();
            --this->sky_burst_;
        }

        // headlights may not be spawned yet; wait until they show up
        std::vector<rendering::LightPtr> headlights;
        for (uint32_t i = 0; i < scene->LightCount(); ++i) {
            rendering::LightPtr light = scene->LightByIndex(i);
            if (light && name_has(light->Name(), "headlight")) {
                headlights.push_back(light);
            }
        }
        if (headlights.empty() && first) return;

        if (time_changed || day != this->day_ || first) {
            this->ApplyLights(scene, headlights, day);
            this->day_ = day;
            ROS_INFO("[light_switch] time=%.2fh (%s): sun=%s headlights=%u",
                     time, day ? "day" : "night", day ? "on" : "off",
                     static_cast<unsigned>(headlights.size()));
        }

        this->time_ = time;
        this->applied_ = true;

        if (this->publish_) this->PublishState(time, day);
    }

    void PublishSky(double _time) {
        if (!this->sky_pub_) return;
        msgs::Sky msg;
        msg.set_time(_time);
        msg.set_sunrise(sunrise());
        msg.set_sunset(sunset());
        this->sky_pub_->Publish(msg);
    }

    void ApplyLights(rendering::ScenePtr _scene,
                     const std::vector<rendering::LightPtr>& _headlights,
                     bool _day) {
        // sun: on during the day, off at night
        rendering::LightPtr sun = _scene->LightByName("sun");
        if (sun) {
            sun->SetVisible(_day);
            sun->SetDiffuseColor(_day ? ignition::math::Color(0.9, 0.9, 0.9)
                                      : ignition::math::Color(0.0, 0.0, 0.0));
        }

        // headlights: on at night, off during the day
        for (const rendering::LightPtr& light : _headlights) {
            light->SetVisible(!_day);
        }

        // bulbs glow at night
        this->SetBulbEmissive(_scene->WorldVisual(), _day);
    }

    void PublishState(double _time, bool _day) {
        std_msgs::Float64 msg;
        msg.data = _time;
        this->state_pub_.publish(msg);
        ros::param::set("/sky_time_current", _time);
        ros::param::set("/sky_is_day", _day);
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

    double time_ = 0.0;
    bool day_ = true;
    bool applied_ = false;
    bool publish_ = false;
    int sky_burst_ = kSkyBurst;
    ros::WallTime last_sky_;
    std::unique_ptr<ros::NodeHandle> nh_;
    ros::Publisher state_pub_;
    transport::NodePtr node_;
    transport::PublisherPtr sky_pub_;
    gazebo::event::ConnectionPtr conn_;
};

GZ_REGISTER_VISUAL_PLUGIN(LightSwitchVisualPlugin)

}  // namespace gazebo
