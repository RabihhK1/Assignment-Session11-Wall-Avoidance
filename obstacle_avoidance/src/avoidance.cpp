#include "obstacle_avoidance/avoidance.hpp"
#include <cmath>

Avoidance::Avoidance()
: Node("avoidance")
{
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&Avoidance::laser_callback, this, std::placeholders::_1));
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker_array", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), std::bind(&Avoidance::timer_callback, this)); // 20 Hz

    // Initialize cmd_vel message
    cmd_vel_msg_.linear.x = 0.5;
    cmd_vel_msg_.angular.z = 0.0;
}

void Avoidance::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Store the laser scan data for processing in the timer callback
    laser_scan_ = msg;
}

void Avoidance::timer_callback()
{
    if (laser_scan_) {
        calculate_forces();
        publish_cmd_vel();
        publish_markers();
    }
}

void Avoidance::calculate_forces()
{
    repulsive_force_.clear();
    resultant_force_.x = 0;
    resultant_force_.y = 0;
    resultant_force_.z = 0;

    // Calculate repulsive forces
    for (size_t i = 0; i < laser_scan_->ranges.size(); ++i) {
        if (laser_scan_->ranges[i] < laser_scan_->range_max) {
            float angle = laser_scan_->angle_min + i * laser_scan_->angle_increment;
            geometry_msgs::msg::Point force = calculate_repulsive_force(laser_scan_->ranges[i], angle);
            repulsive_force_.push_back(force);
            resultant_force_.x += force.x;
            resultant_force_.y += force.y;
        }
    }

    // Calculate attractive force
    attractive_force_ = calculate_attractive_force();
    resultant_force_.x += attractive_force_.x;
    resultant_force_.y += attractive_force_.y;

    // Modify cmd_vel_msg_ based on resultant force
    cmd_vel_msg_.linear.x = 0.5;  // Example value, update based on force calculations
    cmd_vel_msg_.angular.z = std::atan2(resultant_force_.y, resultant_force_.x);
}

geometry_msgs::msg::Point Avoidance::calculate_repulsive_force(float range, float angle)
{
    geometry_msgs::msg::Point force;
    float strength = 1.0 / (range * range); // Repulsive force strength decreases with distance
    force.x = -strength * std::cos(angle);
    force.y = -strength * std::sin(angle);
    force.z = 0.0;
    return force;
}

geometry_msgs::msg::Point Avoidance::calculate_attractive_force()
{
    geometry_msgs::msg::Point force;
    force.x = 1.0; // Attractive force towards the goal in the positive x direction
    force.y = 0.0;
    force.z = 0.0;
    return force;
}

void Avoidance::publish_cmd_vel()
{
    cmd_vel_pub_->publish(cmd_vel_msg_);
}

void Avoidance::publish_markers()
{
    marker_array_.markers.clear();
    int marker_id = 0;

    // Add repulsive forces to marker array
    for (const auto& point : repulsive_force_) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->now();
        marker.ns = "repulsive_force";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        geometry_msgs::msg::Point start;
        start.x = 0;
        start.y = 0;
        start.z = 0;

        geometry_msgs::msg::Point end;
        end.x = point.x;
        end.y = point.y;
        end.z = point.z;

        marker.points.push_back(start);
        marker.points.push_back(end);

        marker_array_.markers.push_back(marker);
    }

    // Add attractive force to marker array
    visualization_msgs::msg::Marker attractive_marker;
    attractive_marker.header.frame_id = "base_link";
    attractive_marker.header.stamp = this->now();
    attractive_marker.ns = "attractive_force";
    attractive_marker.id = marker_id++;
    attractive_marker.type = visualization_msgs::msg::Marker::ARROW;
    attractive_marker.action = visualization_msgs::msg::Marker::ADD;
    attractive_marker.pose.orientation.w = 1.0;
    attractive_marker.scale.x = 0.2;
    attractive_marker.scale.y = 0.04;
    attractive_marker.scale.z = 0.04;
    attractive_marker.color.a = 1.0; // Don't forget to set the alpha!
    attractive_marker.color.r = 0.0;
    attractive_marker.color.g = 0.0;
    attractive_marker.color.b = 1.0;

    geometry_msgs::msg::Point attractive_start;
    attractive_start.x = 0;
    attractive_start.y = 0;
    attractive_start.z = 0;

    geometry_msgs::msg::Point attractive_end;
    attractive_end.x = attractive_force_.x;
    attractive_end.y = attractive_force_.y;
    attractive_end.z = attractive_force_.z;

    attractive_marker.points.push_back(attractive_start);
    attractive_marker.points.push_back(attractive_end);

    marker_array_.markers.push_back(attractive_marker);

    // Add resultant force to marker array
    visualization_msgs::msg::Marker resultant_marker;
    resultant_marker.header.frame_id = "base_link";
    resultant_marker.header.stamp = this->now();
    resultant_marker.ns = "resultant_force";
    resultant_marker.id = marker_id++;
    resultant_marker.type = visualization_msgs::msg::Marker::ARROW;
    resultant_marker.action = visualization_msgs::msg::Marker::ADD;
    resultant_marker.pose.orientation.w = 1.0;
    resultant_marker.scale.x = 0.3;
    resultant_marker.scale.y = 0.06;
    resultant_marker.scale.z = 0.06;
    resultant_marker.color.a = 1.0; // Don't forget to set the alpha!
    resultant_marker.color.r = 0.0;
    resultant_marker.color.g = 1.0;
    resultant_marker.color.b = 0.0;

    geometry_msgs::msg::Point resultant_start;
    resultant_start.x = 0;
    resultant_start.y = 0;
    resultant_start.z = 0;

    geometry_msgs::msg::Point resultant_end;
    resultant_end.x = resultant_force_.x;
    resultant_end.y = resultant_force_.y;
    resultant_end.z = resultant_force_.z;

    resultant_marker.points.push_back(resultant_start);
    resultant_marker.points.push_back(resultant_end);

    marker_array_.markers.push_back(resultant_marker);

    marker_pub_->publish(marker_array_);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Avoidance>());
    rclcpp::shutdown();
    return 0;
}
