#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"


class LimoController : public rclcpp::Node {
public:

    LimoController() : Node("limo_controller", rclcpp::NodeOptions()) {
        // Declare Parameters and get param values
        declare_parameter("angleThreshold", 0.0);
        declare_parameter("distThreshold", 0.0);
        declare_parameter("xGoal", 0.0);
        declare_parameter("yGoal", 0.0);
        declare_parameter("angleGoal", 0.0);
        declare_parameter("coeffProp", 0.0);
        declare_parameter("coeffInt", 0.0);
        declare_parameter("coeffDeriv", 0.0);

        angleThreshold = get_parameter("angleThreshold").as_double();
        distThreshold = get_parameter("distThreshold").as_double();
        xGoal = get_parameter("xGoal").as_double();
        yGoal = get_parameter("yGoal").as_double();
        destAngle = get_parameter("angleGoal").as_double();
        coeffProp = get_parameter("coeffProp").as_double();
        coeffInt = get_parameter("coeffInt").as_double();
        coeffDeriv = get_parameter("coeffDeriv").as_double();

        // Topic declarations
        cmdVelPublisher = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", 10); 
        odomSubscriber = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&LimoController::run_robot, this, 
            std::placeholders::_1));
    }

private:
    // Main loop 
    void run_robot(const nav_msgs::msg::Odometry::SharedPtr msg) {
        get_odom(msg);
        float direction = calculate_direction();
        if(!turnToDestination) {
            if (turn(direction)) {
                turnToDestination = true;
                reset();
            }
        }
        else if (!moveToDirection) {
            if (move(xGoal, yGoal)) {
                moveToDirection = true;
                reset();
            }
        }
        else if (!turnAtDestination) {
            if (turn(destAngle)) {
                turnAtDestination = true;
            }
        }
        else {
            rclcpp::shutdown();
            return;
        }
    }

    // Robot movement functions
    bool turn(const float &goal_dir) {
        error = goal_dir - angleCur;
        integralError += error;
        derivative = error - prevError;            
        
        controlEffort = coeffProp * error + coeffInt * integral + coeffDeriv * 
            derivative;
        prevError = error;
        
        if (abs(angleCur - goal_dir) > angleThreshold) {
            turnMsg.angular.z = controlEffort;
            cmdVelPublisher->publish(turnMsg);
            return false;
        }

        turnMsg.angular.z = 0.0;
        cmdVelPublisher->publish(turnMsg);
        return true;
    }

    bool move(const float &xGoal_pos, const float &yGoal_pos) {
        error = std::sqrt(
            std::pow(xGoal_pos - xCur, 2) + std::pow(yGoal_pos - yCur, 2));
        integralError += error;
        derivative = error - prevError;  

        controlEffort = coeffProp * error + coeffInt * integral + coeffDeriv * 
            derivative;
        prevError = error;

        if (std::sqrt(std::pow(xGoal_pos - xCur, 2) + 
                std::pow(yGoal_pos - yCur, 2)) > distThreshold) {
            moveMsg.linear.x = controlEffort;
            cmdVelPublisher->publish(moveMsg);
            return false;
        }

        moveMsg.linear.x = 0.0;
        cmdVelPublisher->publish(moveMsg);
        return true;
    }

    // Util methods
    void reset() {
        error = 0.0;
        prevError = 0.0;
        integralError = 0.0;
        derivative = 0.0;
        controlEffort = 0.0;
    }

    void get_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        xCur = msg->pose.pose.position.x;
        yCur = msg->pose.pose.position.y;
        angleCur = conv_rad(msg->pose.pose.orientation.z, 
            msg->pose.pose.orientation.w);
        RCLCPP_INFO(this->get_logger(), 
            "Odometry - Position (x: %.4f, y: %.4f) - Angle (theta: %.8f)", 
            xCur, yCur, angleCur);
    }

    float conv_rad(const float &z, const float &w) {
        float angle = 2 * atan2(z, w);
        return angle;
    }

    float calculate_direction() {
        float dir = atan((yGoal - yCur) / (xGoal - xCur));
        if (xGoal < 0) {
            if (dir < 0) {
                dir = M_PI + dir;
            }
            else {
                dir = -M_PI + dir;
            }
        }
        return dir;
    }

    // Flow Variables
    bool turnToDestination = false;
    bool moveToDirection = false;
    bool turnAtDestination = false;

    // Goal Variables
    float destAngle;
    float xGoal;
    float yGoal;

    // Goal Tolerances
    float angleThreshold;
    float distThreshold;

    // PID Control
    float coeffProp;
    float coeffInt;
    float coeffDeriv;

    float error = 0.0;
    float prevError = 0.0;
    float integralError = 0.0;
    float derivative = 0.0;
    float controlEffort = 0.0;
    float integral = 0.0;

    // Current positions
    float xCur;
    float yCur;
    float distCur;
    float angleCur;

    // Messages
    geometry_msgs::msg::Twist moveMsg = geometry_msgs::msg::Twist();
    geometry_msgs::msg::Twist turnMsg = geometry_msgs::msg::Twist();

    // Comm Instances
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPublisher;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSubscriber;

};

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LimoController>());
    rclcpp::shutdown();
    
    return 0;

}