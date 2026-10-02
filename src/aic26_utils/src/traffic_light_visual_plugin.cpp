#include <mutex>
#include <queue>
#include <string>
#include <functional>

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

        std::lock_guard<std::mutex> lock(this->queue_mutex_);
        this->material_queue_.push({_msg->light_name, _msg->material_name});
    }

    void OnPreRender() {
        std::queue<std::pair<std::string, std::string>> local_queue;
        {
            std::lock_guard<std::mutex> lock(this->queue_mutex_);
            std::swap(local_queue, this->material_queue_);
        }

        while (!local_queue.empty()) {
            auto [light_name, material_name] = local_queue.front();
            local_queue.pop();

            rendering::ScenePtr scene = rendering::get_scene();
            if (!scene) continue;

            const std::string target_name = this->model_name_ + "::gantry::" + light_name;
            rendering::VisualPtr target = scene->GetVisual(target_name);
            if (!target) {
                ROS_WARN("[traffic_light_visual_plugin] visual not found: %s",
                         target_name.c_str());
                continue;
            }

            target->SetMaterial(material_name);
        }
    }

    rendering::VisualPtr visual_;
    std::string model_name_;
    std::unique_ptr<ros::NodeHandle> nh_;
    ros::Subscriber sub_;

    std::mutex queue_mutex_;
    std::queue<std::pair<std::string, std::string>> material_queue_;

    gazebo::event::ConnectionPtr pre_render_conn_;
};

GZ_REGISTER_VISUAL_PLUGIN(TrafficLightVisualPlugin)

}  // namespace gazebo