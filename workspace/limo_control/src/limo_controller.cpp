#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

class LimoController : public rclcpp::Node {

public:

    LimoController() : Node("limo_controller") {
        // Service declarations
        params_client = std::make_shared<rclcpp::AsyncParametersClient>(this, "/params_node");

        // Topic declarations
        cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); 
        status_publisher = this->create_publisher<std_msgs::msg::String>("status", 10);
        odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&LimoController::odom_callback, this, std::placeholders::_1));
        
        // Callback timer
        timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&LimoController::run_robot, this));
        
        // Param client
        params_client->wait_for_service();
        auto future = params_client->get_parameters(
            {"x_goal", "y_goal", "theta_goal", "d_x", "d_theta", "d_x_slow", "d_theta_slow", "threshold_dist", "threshold_theta"},
            std::bind(&LimoController::param_callback, this, std::placeholders::_1));

        // Initial messages send
        std_msgs::msg::String status_msg = std_msgs::msg::String();
        status_msg.data = "unready";
        status_publisher->publish(status_msg);
    }

private:

    // Main loop 

    void run_robot() {

        // RCLCPP_INFO(this->get_logger(), "Turn to destination: %d", turn_to_destination);
        if(!turn_to_destination) {
            if (turn_until(dest_angle)) {
                turn_to_destination = true;
                status_publisher->publish(record_first_msg);
            }
            status_publisher->publish(unready_msg);
        }
        else if (!move_to_destination) {
            if (move_until(x_goal, y_goal)) {
                move_to_destination = true;
            }
            status_publisher->publish(unready_msg);
        }
        else if (!turn_at_destination) {
            if (turn_until(theta_goal)) {
                turn_at_destination = true;
                status_publisher->publish(record_final_msg);
            }
            status_publisher->publish(unready_msg);
        }
        else if (turn_to_destination && move_to_destination && turn_at_destination) {
            status_publisher->publish(ready_msg);
        }

    }

    // Robot movement functions

    bool move_until(const float &x, const float &y) {

        RCLCPP_INFO(this->get_logger(), "X cur: %.4f, Y cur: %.4f", x_cur, y_cur);

        if (x == 0 && y == 0) {
            cmd_vel_publisher->publish(stop_msg);
            return true;
        }
        else if (x == 0 && y < 0 && y_cur <= y) {
            cmd_vel_publisher->publish(stop_msg);
               return true;
        }
        else if (x == 0 && y > 0 && y_cur >= y) {
            cmd_vel_publisher->publish(stop_msg);
            return true;
        }
        else if (x > 0 && y == 0 && x_cur >= x) {
            cmd_vel_publisher->publish(stop_msg);
            return true;
        }
        else if (x < 0 && y == 0 && x_cur <= x) {
            cmd_vel_publisher->publish(stop_msg);
            return true;
        }

        // Stop if current position is over the goal position

        else if (x > 0 && y > 0 && x_cur >= x && y_cur >= y) {
            cmd_vel_publisher->publish(stop_msg);
            return true;
        }
        else if (x > 0 && y < 0 && x_cur >= x && y_cur <= y){
            cmd_vel_publisher->publish(stop_msg);
            return true;
        }
        else if (x < 0 && y > 0 && x_cur <= x && y_cur >= y) {
            cmd_vel_publisher->publish(stop_msg);
            return true;
        }
        else if (x < 0 && y < 0 && x_cur <= x && y_cur <= y) {
            cmd_vel_publisher->publish(stop_msg);
            return true;
        }

        // Continue moving forward if not
        if (abs(y_goal - y_cur) >= threshold_dist|| abs(x_goal - x_cur) >= threshold_dist) {
            cmd_vel_publisher->publish(forward_msg);
        }
        else {
            cmd_vel_publisher->publish(forward_slow_msg);
        }
        
        return false;
    }

    bool turn_until(const float &direction) {
        RCLCPP_INFO(this->get_logger(), "Direction: %.4f", direction);
        RCLCPP_INFO(this->get_logger(), "Theta Diff: %.4f", (theta_cur - direction));
        if (direction > 0) {
            if((direction - theta_cur) >= threshold_theta) {
                cmd_vel_publisher->publish(turn_left_msg);
            }
            else {
                cmd_vel_publisher->publish(turn_left_slow_msg);
            }
            if(theta_cur >= direction) {
                cmd_vel_publisher->publish(stop_msg);
                return true;
            }
        }
        if (direction < 0) {
            if((theta_cur - direction) >= threshold_theta) {
                cmd_vel_publisher->publish(turn_right_msg);
            }
            else {
                cmd_vel_publisher->publish(turn_right_slow_msg);
            }
            if(theta_cur <= direction) {
                cmd_vel_publisher->publish(stop_msg);
                return true;
            }
        }
        if (direction == 0) {
            return true;
        }
        return false;
    }

    // Callback functions

    void param_callback(std::shared_future<std::vector<rclcpp::Parameter>> future) {
        auto result = future.get();
        x_goal = result.at(0).as_double();
        y_goal = result.at(1).as_double();
        theta_goal = result.at(2).as_double();
        d_x = result.at(3).as_double();
        d_theta = result.at(4).as_double();
        d_x_slow = result.at(5).as_double();
        d_theta_slow = result.at(6).as_double();
        threshold_dist = result.at(7).as_double();
        threshold_theta = result.at(8).as_double();

        dest_angle = calculate_direction();
        forward_msg.linear.x = d_x;
        forward_slow_msg.linear.x = d_x_slow;
        stop_msg.linear.x = 0;
        stop_msg.angular.z = 0;
        turn_right_msg.angular.z = -1 * d_theta;
        turn_left_msg.angular.z = d_theta;
        turn_right_slow_msg.angular.z = -1 * d_theta_slow;
        turn_left_slow_msg.angular.z = d_theta_slow;
        ready_msg.data = "ready";
        unready_msg.data = "unready";
        record_first_msg.data = "record_first";
        record_final_msg.data = "record_final";
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        x_cur = msg->pose.pose.position.x;
        y_cur = msg->pose.pose.position.y;
        theta_cur = conv_rad(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
        // RCLCPP_INFO(this->get_logger(), "Odometry - Position (x: %.4f, y: %.4f) - Angle (theta: %.8f)", x_cur, y_cur, theta_cur);
    }

    // Util methods

    float conv_rad(const float &z, const float &w) {
        float angle = 2 * atan2(z, w);
        return angle;
    }

    float calculate_direction() {
        float dir = atan((y_goal - y_cur) / (x_goal - x_cur));
        
        return dir;
    }

    // Variable Declartions

    double dest_angle;

    // Parameters from YAML
    double x_goal;
    double y_goal;
    double theta_goal;
    double d_x;
    double d_theta;
    double d_x_slow;
    double d_theta_slow;
    double threshold_dist;
    double threshold_theta;

    // Current positions
    double x_cur;
    double y_cur;
    double theta_cur;

    // Flags
    bool turn_to_destination = false;
    bool move_to_destination = false;
    bool turn_at_destination = false;

    // Messages
    geometry_msgs::msg::Twist forward_msg = geometry_msgs::msg::Twist();
    geometry_msgs::msg::Twist forward_slow_msg = geometry_msgs::msg::Twist();
    geometry_msgs::msg::Twist stop_msg = geometry_msgs::msg::Twist();
    geometry_msgs::msg::Twist turn_left_msg = geometry_msgs::msg::Twist();
    geometry_msgs::msg::Twist turn_right_msg = geometry_msgs::msg::Twist();
    geometry_msgs::msg::Twist turn_left_slow_msg = geometry_msgs::msg::Twist();
    geometry_msgs::msg::Twist turn_right_slow_msg = geometry_msgs::msg::Twist();
    std_msgs::msg::String ready_msg = std_msgs::msg::String();
    std_msgs::msg::String unready_msg = std_msgs::msg::String();
    std_msgs::msg::String record_first_msg = std_msgs::msg::String();
    std_msgs::msg::String record_final_msg = std_msgs::msg::String();

    // Comm Instances
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
    rclcpp::TimerBase::SharedPtr timer;
    std::shared_ptr<rclcpp::AsyncParametersClient> params_client;

};

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LimoController>());
    rclcpp::shutdown();
    
    return 0;

}