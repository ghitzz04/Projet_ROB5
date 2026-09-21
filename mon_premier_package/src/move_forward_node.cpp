#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;

class MoveForwardNode : public rclcpp::Node
{
public:
  MoveForwardNode()
  : Node("move_forward_node"),
    target_distance_(0.3),   // distance a parcourir en metres (modifiable)
    linear_speed_(0.15),     // vitesse d'avance en m/s
    distance_traveled_(0.0),
    has_start_pose_(false),
    goal_reached_(false)
  {
    // Publisher pour envoyer les commandes de vitesse
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Subscriber pour lire l'odometrie et calculer la distance parcourue
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&MoveForwardNode::odom_callback, this, _1));

    // Timer qui publie une commande de vitesse a intervalle regulier
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&MoveForwardNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Objectif : avancer de %.2f m", target_distance_);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    if (!has_start_pose_) {
      start_x_ = x;
      start_y_ = y;
      has_start_pose_ = true;
    }

    distance_traveled_ = std::sqrt(
      std::pow(x - start_x_, 2) + std::pow(y - start_y_, 2));
  }

  void timer_callback()
  {
    if (!has_start_pose_) {
      // On attend d'avoir recu au moins un message d'odometrie
      return;
    }

    geometry_msgs::msg::Twist cmd;

    if (distance_traveled_ < target_distance_ && !goal_reached_) {
      cmd.linear.x = linear_speed_;
      cmd_vel_pub_->publish(cmd);
      RCLCPP_INFO(this->get_logger(), "Distance parcourue : %.2f m", distance_traveled_);
    } else {
      // Objectif atteint : on arrete le robot
      cmd.linear.x = 0.0;
      cmd_vel_pub_->publish(cmd);

      if (!goal_reached_) {
        RCLCPP_INFO(this->get_logger(), "Objectif atteint ! Distance totale : %.2f m",
                    distance_traveled_);
        goal_reached_ = true;
      }
    }
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double target_distance_;
  double linear_speed_;
  double distance_traveled_;
  double start_x_;
  double start_y_;
  bool has_start_pose_;
  bool goal_reached_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveForwardNode>());
  rclcpp::shutdown();
  return 0;
}