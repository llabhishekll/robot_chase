#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <cmath>

class RobotChase : public rclcpp::Node {
private:
  // ros objects
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel;
  rclcpp::TimerBase::SharedPtr timer_robot_control;

  // tf objects
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;

  // member method
  void timer_robot_control_callback() {

    // parameters
    std::string f_ref = "rick/base_link";
    std::string f_tar = "morty/base_link";

    // fetch transformation
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer->lookupTransform(f_ref, f_tar, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      return;
    }

    // publishing velocity to the topic /cmd_vel
    auto message = geometry_msgs::msg::Twist();

    auto x = t.transform.translation.x;
    auto y = t.transform.translation.y;

    // calculate error
    auto error_distance = std::sqrt(x * x + y * y);
    auto error_yaw = std::atan2(y, x);

    // control structure
    if (error_distance > 0.35) {
      message.angular.z = -0.5 * error_yaw;
      message.linear.x = 0.3 * error_distance;
    } else {
      message.angular.z = 0;
      message.linear.x = 0;
    }

    // publish velocity
    publisher_cmd_vel->publish(message);

    RCLCPP_INFO(this->get_logger(), "Location (X%f, Y%f)", x, y);
    RCLCPP_INFO(this->get_logger(), "(D%f, Y%f)", error_distance, error_yaw);
  }

public:
  RobotChase() : Node("robot_chase_node") {
    // ros objects
    this->publisher_cmd_vel =
        this->create_publisher<geometry_msgs::msg::Twist>("rick/cmd_vel", 1);
    this->timer_robot_control = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&RobotChase::timer_robot_control_callback, this));

    // tf objects
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  }
};

int main(int argc, char *argv[]) {
  // initialize ros, executor and node
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  std::shared_ptr<RobotChase> node = std::make_shared<RobotChase>();

  // add node to executor and spin
  executor.add_node(node);
  executor.spin();

  // shutdown
  rclcpp::shutdown();
  return 0;
}