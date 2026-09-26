#include <ros/ros.h>
#include <std_msgs/String.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <cmath>
#include <cstdlib>
#include <sstream>
#include <string>
#include <vector>

class TrafficLightUnit {
public:
    enum Phase { RED, GREEN, YELLOW };

    TrafficLightUnit(const std::string& name, double offset)
        : name_(name), elapsed_(offset) { normalize(); }

    Phase currentPhase() const {
        if (elapsed_ < red_)         return RED;
        if (elapsed_ < red_ + green_) return GREEN;
        return YELLOW;
    }

    void tick(double dt) { if (!paused_) { elapsed_ += dt; normalize(); } }
    void pause()  { paused_ = true;  }
    void resume() { paused_ = false; }
    void reset()  { elapsed_ = 0.0;  }

    void setDurations(double r, double g, double y) {
        red_ = r; green_ = g; yellow_ = y; normalize();
    }
    void setOffset(double o) { elapsed_ = o; normalize(); }

    Phase lastPhase() const   { return lastPhase_; }
    void  setLastPhase(Phase p) { lastPhase_ = p; }
    const std::string& name() const { return name_; }

private:
    void normalize() {
        const double c = red_ + green_ + yellow_;
        if (c <= 0.0) { elapsed_ = 0.0; return; }
        elapsed_ = std::fmod(elapsed_, c);
        if (elapsed_ < 0.0) elapsed_ += c;
    }

    std::string name_;
    double red_    = 30.0;
    double green_  = 25.0;
    double yellow_ = 5.0;
    double elapsed_ = 0.0;
    bool   paused_  = false;
    Phase  lastPhase_ = RED;
};

class TrafficLightController {
public:
    TrafficLightController(ros::NodeHandle& nh) {

        node_.reset(new gazebo::transport::Node());
        node_->Init();
        pub_ = node_->Advertise<gazebo::msgs::Visual>("/gazebo/aic26-1/visual");
        if (!pub_->WaitForConnection(gazebo::common::Time(60, 0))) {
            ROS_WARN("Connection failed.");
        } else {
            ROS_INFO("Connection get.");
        }

        lights_.emplace_back("traffic_light_1", 0.0);
        lights_.emplace_back("traffic_light_2", 15.0);

        subCtrl_  = nh.subscribe("/traffic_light/timer_control", 10,
                                 &TrafficLightController::timerControlCallback, this);
        subState_ = nh.subscribe("/traffic_light/set_state", 10,
                                 &TrafficLightController::setStateCallback, this);

        for (auto& u : lights_) applyPhase(u, true);

        timer_ = nh.createTimer(ros::Duration(0.1),
                                &TrafficLightController::onTick, this);
    }

private:
    void setLens(const std::string& model, const std::string& lens, bool on) {
        gazebo::msgs::Visual msg;
        msg.set_id(0);
        msg.set_name(model + "::gantry::" + lens + "_lens");
        msg.set_parent_name(model + "::gantry");

        auto* script = msg.mutable_material()->mutable_script();
        script->set_name("traffic_light/" + lens + "_" + (on ? "on" : "off"));
        script->add_uri("model://traffic_light/materials/scripts");
        script->add_uri("model://traffic_light/materials/textures");

        pub_->Publish(msg);
    }

    void applyPhase(TrafficLightUnit& u, bool force = false) {
        const auto p = u.currentPhase();
        if (!force && p == u.lastPhase()) return;

        const std::string& m = u.name();
        setLens(m, "red",    p == TrafficLightUnit::RED);
        setLens(m, "green",  p == TrafficLightUnit::GREEN);
        setLens(m, "yellow", p == TrafficLightUnit::YELLOW);
        u.setLastPhase(p);
    }

    void onTick(const ros::TimerEvent& ev) {
        double dt = (ev.current_real - ev.last_real).toSec();
        if (dt <= 0.0 || dt > 1.0) dt = 0.1;

        for (auto& u : lights_) {
            u.tick(dt);
            applyPhase(u);
        }
    }

    std::vector<TrafficLightUnit*> resolve(const std::string& target) {
        std::vector<TrafficLightUnit*> out;
        if (target == "all") {
            for (auto& u : lights_) out.push_back(&u);
        } else if (target == "1") {
            if (!lights_.empty()) out.push_back(&lights_[0]);
        } else if (target == "2") {
            if (lights_.size() > 1) out.push_back(&lights_[1]);
        } else {
            for (auto& u : lights_)
                if (u.name() == target) { out.push_back(&u); break; }
        }
        return out;
    }

    void timerControlCallback(const std_msgs::String::ConstPtr& msg) {
        std::vector<std::string> t;
        std::stringstream ss(msg->data);
        std::string tok;
        while (std::getline(ss, tok, ':')) t.push_back(tok);
        if (t.empty()) return;

        auto targets = resolve(t.size() > 1 ? t[1] : "all");
        if (targets.empty()) return;

        if (t[0] == "pause") {
            for (auto* u : targets) u->pause();
        } else if (t[0] == "resume") {
            for (auto* u : targets) u->resume();
        } else if (t[0] == "reset") {
            for (auto* u : targets) { u->reset(); applyPhase(*u, true); }
        } else if (t[0] == "set" && t.size() >= 5) {
            for (auto* u : targets) {
                u->setDurations(std::stod(t[2]), std::stod(t[3]), std::stod(t[4]));
                applyPhase(*u, true);
            }
        } else if (t[0] == "offset" && t.size() >= 3) {
            for (auto* u : targets) {
                u->setOffset(std::stod(t[2]));
                applyPhase(*u, true);
            }
        }
    }

    void setStateCallback(const std_msgs::String::ConstPtr& msg) {
        const auto pos = msg->data.find(':');
        if (pos == std::string::npos) return;

        const std::string target = msg->data.substr(0, pos);
        const std::string state  = msg->data.substr(pos + 1);

        if (state != "red" && state != "green" && state != "yellow" && state != "off")
            return;

        for (auto* u : resolve(target)) {
            u->pause();
            const std::string& m = u->name();
            setLens(m, "red",    state == "red");
            setLens(m, "green",  state == "green");
            setLens(m, "yellow", state == "yellow");
        }
    }

    gazebo::transport::NodePtr      node_;
    gazebo::transport::PublisherPtr pub_;
    std::vector<TrafficLightUnit>   lights_;
    ros::Subscriber subCtrl_, subState_;
    ros::Timer      timer_;
};

int main(int argc, char** argv) {

    ROS_INFO("GAZEBO_MASTER_URI=%s", getenv("GAZEBO_MASTER_URI"));

    ros::init(argc, argv, "traffic_light_controller");
    ros::NodeHandle nh;
    TrafficLightController controller(nh);
    ros::spin();

    return 0;
}