// Gazebo model plugin that makes every joint behave like a hobby servo (MG90S):
//   torque = clamp( kp*(target - angle) - kd*velocity , +-max_torque )
// Targets arrive on  /recon/joint_commands  (std_msgs/Float64MultiArray, radians, same order as the
// <joint> tags in the plugin SDF).  Joint states are published on /joint_states.
#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace recon_sim
{

class ReconServoPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    node_ = gazebo_ros::Node::Get(sdf);
    kp_ = sdf->HasElement("kp") ? sdf->Get<double>("kp") : 1.0;
    kd_ = sdf->HasElement("kd") ? sdf->Get<double>("kd") : 0.006;
    max_torque_ = sdf->HasElement("max_torque") ? sdf->Get<double>("max_torque") : 0.25;
    double state_rate = sdf->HasElement("state_rate") ? sdf->Get<double>("state_rate") : 50.0;
    state_period_ = 1.0 / state_rate;

    if (sdf->HasElement("joint")) {
      for (auto e = sdf->GetElement("joint"); e; e = e->GetNextElement("joint")) {
        std::string name = e->Get<std::string>();
        auto joint = model->GetJoint(name);
        if (!joint) {
          RCLCPP_ERROR(node_->get_logger(), "recon_servo: joint '%s' not found", name.c_str());
          continue;
        }
        joints_.push_back(joint);
        names_.push_back(name);
      }
    }
    target_.assign(joints_.size(), 0.0);
    world_ = model->GetWorld();

    sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/recon/joint_commands", rclcpp::QoS(1),
      [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t i = 0; i < std::min(msg->data.size(), target_.size()); ++i) {
          target_[i] = msg->data[i];
        }
      });
    pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(10));

    update_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&ReconServoPlugin::OnUpdate, this, std::placeholders::_1));
    RCLCPP_INFO(
      node_->get_logger(), "recon_servo: %zu joints, kp=%.3f kd=%.4f max_torque=%.3f N.m",
      joints_.size(), kp_, kd_, max_torque_);
  }

private:
  void OnUpdate(const gazebo::common::UpdateInfo & info)
  {
    std::vector<double> target;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      target = target_;
    }
    for (size_t i = 0; i < joints_.size(); ++i) {
      const double q = joints_[i]->Position(0);
      const double qd = joints_[i]->GetVelocity(0);
      double tau = kp_ * (target[i] - q) - kd_ * qd;
      tau = std::max(-max_torque_, std::min(max_torque_, tau));
      joints_[i]->SetForce(0, tau);
    }

    const double t = info.simTime.Double();
    if (t - last_state_ >= state_period_) {
      last_state_ = t;
      sensor_msgs::msg::JointState js;
      js.header.stamp = rclcpp::Time(info.simTime.sec, info.simTime.nsec, RCL_ROS_TIME);
      js.name = names_;
      for (auto & j : joints_) {
        js.position.push_back(j->Position(0));
        js.velocity.push_back(j->GetVelocity(0));
      }
      pub_->publish(js);
    }
  }

  gazebo_ros::Node::SharedPtr node_;
  gazebo::physics::WorldPtr world_;
  std::vector<gazebo::physics::JointPtr> joints_;
  std::vector<std::string> names_;
  std::vector<double> target_;
  std::mutex mutex_;
  double kp_{1.0}, kd_{0.006}, max_torque_{0.25};
  double state_period_{0.02}, last_state_{-1.0};
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
  gazebo::event::ConnectionPtr update_;
};

GZ_REGISTER_MODEL_PLUGIN(ReconServoPlugin)

}  // namespace recon_sim
