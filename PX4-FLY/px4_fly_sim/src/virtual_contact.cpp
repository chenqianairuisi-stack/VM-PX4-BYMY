// Gazebo-only evaluation: report a near pass, never send guidance.
#include <cmath>
#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

namespace px4_fly_sim {
class VirtualContact final : public gazebo::WorldPlugin {
 public:
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override {
    world_ = world;
    ros_ = gazebo_ros::Node::Get(sdf);
    threshold_ = sdf->HasElement("threshold") ? sdf->Get<double>("threshold") : 3.5;
    if (!std::isfinite(threshold_) || threshold_ < 0.5) {
      RCLCPP_ERROR(ros_->get_logger(), "Virtual contact threshold must be >= 0.5 m");
      return;
    }
    event_pub_ = ros_->create_publisher<std_msgs::msg::Bool>(
        "/tracker/drone/virtual_contact", rclcpp::QoS(1).transient_local().reliable());
    distance_pub_ = ros_->create_publisher<std_msgs::msg::Float64>(
        "/evaluation/drone_distance", rclcpp::QoS(1).best_effort());
    Reset();
    connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&VirtualContact::Update, this));
  }

  void Reset() override {
    last_time_ = -1.0;
    last_publish_ = -1.0;
    last_inside_ = false;
    Publish(false);
  }

 private:
  void Publish(bool inside) {
    if (!event_pub_) return;
    std_msgs::msg::Bool event;
    event.data = inside;
    event_pub_->publish(event);
  }

  void Update() {
    const double now = world_->SimTime().Double();
    if (now < last_time_) Reset();
    last_time_ = now;
    auto target = world_->ModelByName("target");
    auto tracker = world_->ModelByName("tracker");
    if (!target || !tracker) return;
    auto target_base = target->GetLink("base_link");
    auto tracker_base = tracker->GetLink("base_link");
    if (!target_base || !tracker_base) return;
    const double distance = (target_base->WorldPose().Pos() -
                             tracker_base->WorldPose().Pos()).Length();
    const bool airborne = target_base->WorldPose().Pos().Z() > 1.8 &&
                          tracker_base->WorldPose().Pos().Z() > 1.8;
    const bool inside = airborne && distance <= threshold_;
    if (now - last_publish_ >= 0.05 || inside != last_inside_) {
      std_msgs::msg::Float64 metric;
      metric.data = distance;
      distance_pub_->publish(metric);
      Publish(inside);
      if (inside && !last_inside_)
        RCLCPP_INFO(ros_->get_logger(), "Virtual drone contact: %.3f m", distance);
      last_publish_ = now;
      last_inside_ = inside;
    }
  }

  gazebo::physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr ros_;
  gazebo::event::ConnectionPtr connection_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr event_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;
  double threshold_{3.5}, last_time_{-1.0}, last_publish_{-1.0};
  bool last_inside_{false};
};
GZ_REGISTER_WORLD_PLUGIN(VirtualContact)
}  // namespace px4_fly_sim
