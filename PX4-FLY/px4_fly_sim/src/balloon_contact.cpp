// Simulator-only geometric contact. Target truth never goes to guidance.
#include <algorithm>
#include <array>
#include <functional>
#include <limits>
#include <string>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_ros/node.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

namespace px4_fly_sim {
class BalloonContact final : public gazebo::WorldPlugin {
 public:
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override {
    world_ = world;
    ros_ = gazebo_ros::Node::Get(sdf);
    burst_pub_ = ros_->create_publisher<std_msgs::msg::Bool>(
        "/tracker/balloon/burst", rclcpp::QoS(1).transient_local().reliable());
    distance_pub_ = ros_->create_publisher<std_msgs::msg::Float64>(
        "/evaluation/balloon_contact_distance", rclcpp::QoS(1).best_effort());
    transport_.reset(new gazebo::transport::Node());
    transport_->Init(world->Name());
    visual_pub_ = transport_->Advertise<gazebo::msgs::Visual>("~/visual");
    radius_ = sdf->HasElement("balloon_radius") ? sdf->Get<double>("balloon_radius") : .22;
    Reset();
    connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&BalloonContact::Update, this));
  }

  void Reset() override {
    burst_ = false;
    have_previous_ = false;
    last_time_ = -1.;
    last_publish_ = -1.;
    visuals_visible_ = true;
    last_visual_update_ = -1.;
    std_msgs::msg::Bool message;
    message.data = false;
    if (burst_pub_) burst_pub_->publish(message);
    ShowTargetVisuals(true);
  }

 private:
  void ShowVisual(const char *name, bool visible) {
    if (!visual_pub_) return;
    gazebo::msgs::Visual visual;
    visual.set_name(std::string("target::base_link::") + name);
    visual.set_parent_name("target::base_link");
    visual.set_visible(visible);
    visual.set_transparency(visible ? 0.0 : 1.0);
    visual_pub_->Publish(visual);
  }

  void ShowTargetVisuals(bool visible) {
    ShowVisual("tracking_balloon_visual", visible && !burst_);
    ShowVisual("tracking_tether_visual", visible);
  }

  struct TrackedPoint {
    ignition::math::Vector3d relative;
    double radius;
  };

  static double SweptDistance(const ignition::math::Vector3d &previous,
                              const ignition::math::Vector3d &current) {
    const auto delta = current - previous;
    const double length = delta.SquaredLength();
    const double fraction = length > 1e-12
        ? std::clamp(-previous.Dot(delta) / length, 0.0, 1.0) : 0.0;
    return (previous + delta * fraction).Length();
  }

  void Update() {
    const double now = world_->SimTime().Double();
    if (now < last_time_) Reset();
    if (burst_) {
      if (now-last_publish_ > .5) {
        auto target = world_->ModelByName("target");
        auto base = target ? target->GetLink("base_link") : nullptr;
        visuals_visible_ = base && base->WorldPose().Pos().Z() > 1.35;
        ShowTargetVisuals(visuals_visible_);
        last_publish_ = now;
      }
      last_time_ = now;
      return;
    }
    auto target = world_->ModelByName("target");
    auto tracker = world_->ModelByName("tracker");
    if (!target || !tracker) return;
    auto base = tracker->GetLink("base_link");
    auto camera = tracker->GetLink("front_camera_link");
    auto target_base = target->GetLink("base_link");
    if (!target_base || !base || !camera) return;
    // Do not sweep across a paused/reset simulation interval. This also keeps
    // an invalid spawn pose from producing a false hit on the first frame.
    if (last_time_ >= 0.0 && now - last_time_ > 0.10) {
      have_previous_ = false;
      last_time_ = now;
      return;
    }
    const auto target_pose = target_base->WorldPose();
    const auto balloon_position = target_pose.Pos() + target_pose.Rot().RotateVector(
        ignition::math::Vector3d(0, 0, -1.05));
    // Below this height the sphere would extend through the ground. It is
    // rendered only in flight; its geometry never provides a ground contact.
    const bool airborne_visual = target_pose.Pos().Z() > 1.35;
    if (airborne_visual != visuals_visible_ || now - last_visual_update_ > .5) {
      ShowTargetVisuals(airborne_visual);
      visuals_visible_ = airborne_visual;
      last_visual_update_ = now;
    }
    if (!airborne_visual) {
      have_previous_ = false;
      last_time_ = now;
      return;
    }
    const auto camera_pose = camera->WorldPose();
    const auto base_pose = base->WorldPose();
    // Check the body, camera housing, and nose. The envelope radius accounts
    // for the physical tracker geometry around each sampled center.
    const std::array<TrackedPoint, 3> points = {{
      {base_pose.Pos() - balloon_position, .28},
      {camera_pose.Pos() - balloon_position, .10},
      {camera_pose.Pos() + camera_pose.Rot().RotateVector(
          ignition::math::Vector3d(.04, 0, 0)) - balloon_position, .08},
    }};
    if (have_previous_) {
      for (size_t i = 0; i < points.size(); ++i) {
        if ((points[i].relative - previous_[i]).Length() > 20.0) {
          have_previous_ = false;
          break;
        }
      }
    }
    double distance = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < points.size(); ++i) {
      const double point_distance = have_previous_
          ? SweptDistance(previous_[i], points[i].relative)
          : points[i].relative.Length();
      distance = std::min(distance, point_distance - points[i].radius);
      previous_[i] = points[i].relative;
    }
    have_previous_ = true;
    last_time_ = now;
    if (now-last_publish_ >= .05) {
      std_msgs::msg::Float64 metric;
      metric.data = distance;
      distance_pub_->publish(metric);
      last_publish_ = now;
    }
    if (distance <= radius_) {
      burst_ = true;
      // The balloon bursts; the one-metre tether stays attached to the target.
      ShowTargetVisuals(visuals_visible_);
      std_msgs::msg::Float64 metric;
      metric.data = distance;
      distance_pub_->publish(metric);
      std_msgs::msg::Bool message;
      message.data = true;
      burst_pub_->publish(message);
      RCLCPP_INFO(ros_->get_logger(), "Balloon geometric contact confirmed: %.4f m", distance);
    }
  }

  gazebo::physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr ros_;
  gazebo::transport::NodePtr transport_;
  gazebo::transport::PublisherPtr visual_pub_;
  gazebo::event::ConnectionPtr connection_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr burst_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr distance_pub_;
  std::array<ignition::math::Vector3d, 3> previous_;
  bool burst_{false}, have_previous_{false};
  bool visuals_visible_{true};
  double last_visual_update_{-1.};
  double radius_{.22}, last_time_{-1}, last_publish_{-1};
};
GZ_REGISTER_WORLD_PLUGIN(BalloonContact)
}  // namespace px4_fly_sim
