#ifndef AVOIDANCE_HPP_
#define AVOIDANCE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class Avoidance : public rclcpp::Node
{
public:
    Avoidance();

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void timer_callback();
    void publish_cmd_vel();
    void publish_markers();
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::Twist cmd_vel_msg_;
    visualization_msgs::msg::MarkerArray marker_array_;
    sensor_msgs::msg::LaserScan::SharedPtr laser_scan_;
    std::vector<geometry_msgs::msg::Vector3> repulsive_force_;
    geometry_msgs::msg::Vector3 attractive_force_;
    geometry_msgs::msg::Vector3 resultant_force_;

    void calculate_forces();
    geometry_msgs::msg::Vector3 calculate_repulsive_force(float range, float angle);
    geometry_msgs::msg::Vector3 calculate_attractive_force();
};

#endif  // AVOIDANCE_HPP_
