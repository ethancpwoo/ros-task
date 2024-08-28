#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class LimoController : public rclcpp::Node {

public:

    LimoController() : Node("limo_controller") {
        cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); 
        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&LimoController::odom_callback, this, std::placeholders::_1));
        
        //timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&LimoController::move_forward, this));

        RCLCPP_INFO(this->get_logger(), "Limo Controller Node has been started.");
    }

private:

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Odometry - Position (x: %.2f, y: %.2f)", msg->pose.pose.position.x, msg->pose.pose.position.y);
    }

    void move_forward() {
        geometry_msgs::msg::Twist vel_msg = geometry_msgs::msg::Twist();
        vel_msg.linear.x = 0.1;
        vel_msg.angular.z = 0.0; 

        cmd_vel_publisher->publish(vel_msg);
        RCLCPP_INFO(this->get_logger(), "Publishing velocity command - Linear: %.2f, Angular: %.2f", vel_msg.linear.x, vel_msg.angular.z);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::TimerBase::SharedPtr timer;

};

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LimoController>());
    rclcpp::shutdown();
    
    return 0;

}