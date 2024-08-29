#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class LimoController : public rclcpp::Node {

public:

    LimoController() : Node("limo_controller") {
        cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); 
        status_publisher = this->create_publisher<std_msgs::msg::String>("status", 10);
        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&LimoController::odom_callback, this, std::placeholders::_1));
        
        timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&LimoController::run_robot, this));

        dest_angle = calculate_direction();

        std_msgs::msg::String status_msg = std_msgs::msg::String();
        status_msg.data = "unready";
        status_publisher->publish(status_msg);

        RCLCPP_INFO(this->get_logger(), "dest angle: %.8f", dest_angle);
        RCLCPP_INFO(this->get_logger(), "Limo Controller Node has been started.");
    }

private:

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        x_cur = msg->pose.pose.position.x;
        y_cur = msg->pose.pose.position.y;
        theta_cur = conv_rad(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        RCLCPP_INFO(this->get_logger(), "Odometry - Position (x: %.4f, y: %.4f) - Angle (theta: %.8f)", x_cur, y_cur, theta_cur);
    }

    float conv_rad(const float &z, const float &w) {
        float angle = 2 * atan2(z, w);
        return angle;
    }

    float calculate_direction() {
        float dir = atan((y_goal - y_cur) / (x_goal - x_cur));
        return dir;
    }

    void move_forward() {
        geometry_msgs::msg::Twist vel_msg = geometry_msgs::msg::Twist();
        vel_msg.linear.x = 1.0;
        vel_msg.angular.z = -0.2; 

        cmd_vel_publisher->publish(vel_msg);
        // RCLCPP_INFO(this->get_logger(), "Publishing velocity command - Linear: %.2f, Angular: %.2f", vel_msg.linear.x, vel_msg.angular.z);
    }

    bool move_until(const float &x, const float &y) {
        geometry_msgs::msg::Twist forward_msg = geometry_msgs::msg::Twist();
        geometry_msgs::msg::Twist stop_msg = geometry_msgs::msg::Twist();

        forward_msg.linear.x = 0.5;
        stop_msg.linear.x = 0;

        cmd_vel_publisher->publish(forward_msg);

        if (x == 0 && y == 0) {
            cmd_vel_publisher->publish(stop_msg);
            return true;
        }
        else if (x == 0 && y < 0) {
            if (y_cur <= y) {
                cmd_vel_publisher->publish(stop_msg);
                return true;
            }
            return false;
        }
        else if (x == 0 && y > 0) {
            if (y_cur >= y) {
                cmd_vel_publisher->publish(stop_msg);
                return true;
            }
            return false;
        }
        else if (x > 0 && y == 0) {
            if (x_cur >= x) {
                cmd_vel_publisher->publish(stop_msg);
                return true;
            }
            return false;
        }
        else if (x < 0 && y == 0) {
            if (x_cur <= x) {
                cmd_vel_publisher->publish(stop_msg);
                return true;
            }
            return false;
        }
        else if (x > 0 && y > 0) {
            if (x_cur >= x && y_cur >= y) {
                cmd_vel_publisher->publish(stop_msg);
                return true;
            }
            return false;
        }
        else if (x > 0 && y < 0){
            if (x_cur >= x && y_cur <= y) {
                cmd_vel_publisher->publish(stop_msg);
                return true;
            }
            return false;
        }
        else if (x < 0 && y > 0) {
            if (x_cur <= x && y_cur >= y) {
                cmd_vel_publisher->publish(stop_msg);
                return true;
            }
            return false;

        }
        else if (x < 0 && y < 0) {
            if (x_cur <= x && y_cur <= y) {
                cmd_vel_publisher->publish(stop_msg);
                return true;
            }
            return false;
        }
    }

    bool turn_robot(const float &direction) {
        geometry_msgs::msg::Twist turn_left_msg = geometry_msgs::msg::Twist();
        geometry_msgs::msg::Twist turn_right_msg = geometry_msgs::msg::Twist();
        geometry_msgs::msg::Twist stop_msg = geometry_msgs::msg::Twist();

        turn_left_msg.angular.z = 0.2;
        turn_right_msg.angular.z = -0.2;
        stop_msg.angular.z = 0;

        if (direction > 0) {
            cmd_vel_publisher->publish(turn_left_msg);
            if(theta_cur >= direction) {
                cmd_vel_publisher->publish(stop_msg);
                return true;
            }
            return false;
        }
        if (direction < 0) {
            cmd_vel_publisher->publish(turn_right_msg);
            if(theta_cur <= direction) {
                cmd_vel_publisher->publish(stop_msg);
                return true;
            }
            return false;
        }
        if (direction == 0) {
            return true;
        }
    }

    void run_robot() {

        std_msgs::msg::String status_msg = std_msgs::msg::String();
        if(!turn_to_destination) {
            if (turn_robot(dest_angle)) {
                turn_to_destination = true;
                status_msg.data = "record_first";
                status_publisher->publish(status_msg);
            }
            status_msg.data = "unready";
            status_publisher->publish(status_msg);
        }
        else if (!move_to_destination) {
            if (move_until(x_goal, y_goal)) {
                move_to_destination = true;
            }
            status_msg.data = "unready";
            status_publisher->publish(status_msg);
        }
        else if (!turn_at_destination) {
            if (turn_robot(theta_goal)) {
                turn_at_destination = true;
                status_msg.data = "record_final";
                status_publisher->publish(status_msg);
            }
            status_msg.data = "unready";
            status_publisher->publish(status_msg);
        }
        else if (turn_to_destination && move_to_destination && turn_at_destination) {
            status_msg.data = "ready";
            status_publisher->publish(status_msg);
        }

    }

    float dest_angle;

    float x_goal = 3;
    float y_goal = 3;
    float theta_goal = 3.1415;

    float x_cur;
    float y_cur;
    float theta_cur;

    bool turn_to_destination = false;
    bool move_to_destination = false;
    bool turn_at_destination = false;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::TimerBase::SharedPtr timer;

};

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LimoController>());
    rclcpp::shutdown();
    
    return 0;

}