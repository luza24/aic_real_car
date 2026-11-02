#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/rendering.hh>

#include <ros/ros.h>
#include <aic26_msgs/TrafficLightCommand.h>

namespace gazebo {

class TrafficLightVisualPlugin : public VisualPlugin {
public:
    void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf) override {

        this->visual_ = _visual;
        const std::string scoped = _visual->Name();
        const size_t pos = scoped.find("::");
        this->model_name_ = (pos == std::string::npos) ? scoped : scoped.substr(0, pos);

        // Thread-safe ROS initialization
        static std::mutex init_mutex;
        std::lock_guard<std::mutex> lock(init_mutex);
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = nullptr;
            ros::init(argc, argv, "traffic_light_visual_plugin",
                      ros::init_options::NoSigintHandler);
        }
        this->nh_.reset(new ros::NodeHandle());
        this->sub_ = this->nh_->subscribe(
            "traffic_light/command", 10,
            &TrafficLightVisualPlugin::OnCommand, this
        );

        this->pre_render_conn_ = gazebo::event::Events::ConnectPreRender(
            std::bind(&TrafficLightVisualPlugin::OnPreRender, this));
    }

private:
    void OnCommand(const aic26_msgs::TrafficLightCommand::ConstPtr& _msg) {
        if (!_msg->model_name.empty() && _msg->model_name != this->model_name_) return;

        // Remember the desired material per light. This both coalesces repeated
        // commands and lets us re-apply the state once the render scene has
        // finished building the visual (the world may still be loading when the
        // first command arrives).
        std::lock_guard<std::mutex> lock(this->queue_mutex_);
        this->desired_materials_[_msg->light_name] = _msg->material_name;
        this->pending_dirty_ = true;
    }

    void OnPreRender() {
        std::map<std::string, std::string> desired;
        {
            std::lock_guard<std::mutex> lock(this->queue_mutex_);
            // Only touch the scene when something changed; re-applying every
            // frame would be wasteful. Commands that failed because the visual
            // was not ready yet keep pending_dirty_ set, so they are retried.
            if (!this->pending_dirty_) return;
            desired = this->desired_materials_;
        }

        rendering::ScenePtr scene = rendering::get_scene();
        if (!scene) return;

        bool all_applied = true;
        for (const auto& kv : desired) {
            const std::string& light_name = kv.first;
            const std::string& material_name = kv.second;

            const std::string target_name =
                this->model_name_ + "::gantry::" + light_name;
            rendering::VisualPtr target = scene->GetVisual(target_name);
            if (!target) {
                // The scene is not fully built yet (gzclient still loading the
                // world). Keep the command pending and try again next frame
                // instead of dropping it, otherwise the light would stay dark
                // until the next phase change.
                all_applied = false;
                continue;
            }

            target->SetMaterial(material_name);
        }

        if (all_applied) {
            std::lock_guard<std::mutex> lock(this->queue_mutex_);
            // Only clear the dirty flag if no newer command arrived meanwhile.
            if (this->desired_materials_ == desired) {
                this->pending_dirty_ = false;
            }
        }
    }

    rendering::VisualPtr visual_;
    std::string model_name_;
    std::unique_ptr<ros::NodeHandle> nh_;
    ros::Subscriber sub_;

    std::mutex queue_mutex_;
    std::map<std::string, std::string> desired_materials_;
    bool pending_dirty_ = false;

    gazebo::event::ConnectionPtr pre_render_conn_;
};

GZ_REGISTER_VISUAL_PLUGIN(TrafficLightVisualPlugin)

}  // namespace gazebo
