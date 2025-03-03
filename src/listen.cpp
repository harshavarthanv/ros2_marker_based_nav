#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <set>
#include <map>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include "sensor_msgs/msg/point_cloud.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>

// #include "turtlesim/srv/spawn.hpp"

using namespace std::chrono_literals;
double turn_cmd;
struct BatteryData
{
    std::string color;
    std::string type;

    bool operator<(const BatteryData &other) const
    {
        return std::tie(color, type) < std::tie(other.color, other.type);
    }
};
class FrameListener : public rclcpp::Node
{
public:
    FrameListener()
        : Node("turtle_tf2_frame_listener")
    {
        this->declare_parameter<std::string>("aruco_marker_0", "aruco_marker_0");
        this->declare_parameter<std::string>("aruco_marker_1", "aruco_marker_1");
        this->declare_parameter<std::string>("aruco_marker_2", "aruco_marker_2");
        std::vector<BatteryData> batteries;
        std::map<uint8_t, std::set<uint8_t>> unique_color;
        std::map<uint8_t, std::set<uint8_t>> unique_type;

        tf_buffer_a_ =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_a_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_a_);
        tf_buffer_b_ =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_b_ =
            std::make_shared<tf2_ros::TransformListener>(*tf_buffer_b_);
        subscriptionc_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "mage/advanced_logical_camera/image", rclcpp::SensorDataQoS(),
            std::bind(&FrameListener::handle_turtle_pose, this, std::placeholders::_1));
        publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
        // timer_a_ = this->create_wall_timer(
        //     1s, std::bind(&FrameListener::on_timer1, this));
        subscription_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            "aruco_markers", 10, std::bind(&FrameListener::aruco_callback, this, std::placeholders::_1));
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&FrameListener::odom_callback, this, std::placeholders::_1));
    }

private:
    
    double current_yaw_;
    int desc = 0;
    int key = 0;
    double start_yaw_;
    double target_yaw_;
    bool pose_key_;
    std::vector<float> battery_pose;
    // bool stop_robot ;
    
    void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
    {
        bool stop_robot = false;
        std::string fromFrameRel = "base_footprint";
        std::string toFrameRel = "marker";

        geometry_msgs::msg::TransformStamped t;
        t = tf_buffer_a_->lookupTransform(
            toFrameRel, fromFrameRel,
            tf2::TimePointZero);

        if (t.transform.translation.z <= 0.8)
        {
            stop_robot = true; // Stop checking if any marker is too close
        }

        auto message = geometry_msgs::msg::Twist();
        if (stop_robot)
        {
            for (size_t i = 0; i < msg->poses.size(); ++i)
            {
                if (msg->poses[i].position.z <= 0.8)
                {
                    long int marker_id = msg->marker_ids[i];
                    pose_key_ = false;
                    std::string marker_action;
                    this->get_parameter("aruco_marker_" + std::to_string(marker_id), marker_action);
                    perform_action(marker_action);
                    break;
                }
            }
        }
        else
        {
            // Move forward
            if (desc == 0 && key == 0)
            {
                message.linear.x = 0.1;
                pose_key_ = true;
            }
        }
        publisher_->publish(message);
    }

    void perform_action(const std::string &action)
    {

        auto message = geometry_msgs::msg::Twist();
        if (action == "right_90")
        {

            desc = 1;
            if (key == 0)
            {
                start_yaw_ = current_yaw_;
                if (start_yaw_ < -90 && start_yaw_ > -180)
                {
                    target_yaw_ = start_yaw_ + 270;
                }
                else
                {
                    target_yaw_ = start_yaw_ - 90;
                }
                key = 1;
            }
        }
        else if (action == "left_90")
        {
            desc = 2;
            if (key == 0)
            {
                start_yaw_ = current_yaw_;
                if (start_yaw_ > 90 && start_yaw_ < 180)
                {
                    target_yaw_ = start_yaw_ - 270;
                }
                else
                {
                    target_yaw_ = start_yaw_ + 90;
                }
                key = 1;
            }
            // Turn left 90 degrees
        }
        else if (action == "end")
        {
            desc = 3;
            message.angular.z = 0.0;
            publisher_->publish(message);
            for (const auto &battery : unique_batteries)
            {
                RCLCPP_INFO(this->get_logger(), "Stored Battery: Color = %s, Type = %s", battery.color.c_str(), battery.type.c_str());
            }
            RCLCPP_INFO(this->get_logger(), "Battery 1 Position.x = %f, Position.y = %f, Position.z = %f, Orientatation.x = %f, Orientatation.y = %f, Orientatation.z = %f, Orientatation.w = %f", battery_pose[0], battery_pose[1], battery_pose[2], battery_pose[3], battery_pose[4], battery_pose[5], battery_pose[6]);
            RCLCPP_INFO(this->get_logger(), "Battery 2 Position.x = %f, Position.y = %f, Position.z = %f, Orientatation.x = %f, Orientatation.y = %f, Orientatation.z = %f, Orientatation.w = %f", battery_pose[7], battery_pose[8], battery_pose[9], battery_pose[10], battery_pose[11], battery_pose[12], battery_pose[13]);
            RCLCPP_INFO(this->get_logger(), "Battery 3 Position.x = %f, Position.y = %f, Position.z = %f, Orientatation.x = %f, Orientatation.y = %f, Orientatation.z = %f, Orientatation.w = %f", battery_pose[14], battery_pose[15], battery_pose[16], battery_pose[17], battery_pose[18], battery_pose[19], battery_pose[20]);
            // Stop the robot
            // message has zero velocities by default
        }

        // Stop the robot after the action
        if (action == "right_90" || action == "left_90")
        {
            publisher_->publish(geometry_msgs::msg::Twist()); // Stop the robot
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        double roll, pitch, yaw;
        auto now = std::chrono::steady_clock::now();
        quaternionToEuler(msg->pose.pose.orientation, roll, pitch, yaw);

        current_yaw_ = yaw;
        auto message = geometry_msgs::msg::Twist();
        if (desc == 1 && key == 1)
        {
            start_yaw_ = current_yaw_;
            message.angular.z = -0.3;
            publisher_->publish(message);
            if (target_yaw_ > 90 && target_yaw_ < 180)
            {
                if (start_yaw_ < 180 && start_yaw_ > 90)
                {
                    if ((target_yaw_ - start_yaw_) <= 0)
                    {
                        desc = 0;
                        key = 0;
                    }
                }
            }
            else if ((target_yaw_ - start_yaw_) > 0)
            {
                desc = 0;
                key = 0;
            }
        }
        else if (desc == 2)
        {
            start_yaw_ = current_yaw_;
            message.angular.z = 0.3;
            publisher_->publish(message);
            if (target_yaw_ < -90 && target_yaw_ > -180)
            {
                if (start_yaw_ > -180 && start_yaw_ < -90)
                {
                    if ((target_yaw_ - start_yaw_) >= 0)
                    {
                        desc = 0;
                        key = 0;
                    }
                }
            }
            else if ((target_yaw_ - start_yaw_) <= 0)
            {
                desc = 0;
                key = 0;
            }
        }
    }

    void quaternionToEuler(const geometry_msgs::msg::Quaternion &q, double &roll, double &pitch, double &yaw)
    {
        // Quaternion to RPY conversion
        tf2::Quaternion tf2_quat(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(tf2_quat);
        m.getRPY(roll, pitch, yaw);

        // // Convert from radians to degrees
        roll = roll * 180.0 / M_PI;
        pitch = pitch * 180.0 / M_PI;
        yaw = yaw * 180.0 / M_PI;
    }

    void handle_turtle_pose(const std::shared_ptr<mage_msgs::msg::AdvancedLogicalCameraImage> msg)
    {
       if (msg->part_poses.size() != 0 && pose_key_)
            {
                double roll, pitch, yaw;
                std::this_thread::sleep_for(std::chrono::seconds(1));
                // Store frame names in variables that will be used to
                // compute transformations
                std::string fromFrameRel = "odom";
                std::string toFrameRel = "object";

                geometry_msgs::msg::TransformStamped t;
                t = tf_buffer_b_->lookupTransform(
                    toFrameRel, fromFrameRel,
                    tf2::TimePointZero);

                quaternionToEuler(t.transform.rotation, roll, pitch, yaw);
                int abc = 0;
                for (const auto &battery : unique_batteries)
                {
                    std::size_t colorLength = battery.color.length();
                    if (colorLength == 1 && abc == 0)
                    {
                        battery_pose.push_back(t.transform.translation.x);
                        battery_pose.push_back(t.transform.translation.y);
                        battery_pose.push_back(t.transform.translation.z);
                        battery_pose.push_back(t.transform.rotation.x);
                        battery_pose.push_back(t.transform.rotation.y);
                        battery_pose.push_back(t.transform.rotation.z);
                        battery_pose.push_back(t.transform.rotation.w);
                        abc = 1;
                    }

                    if (colorLength == 2 && abc == 1)
                    {
                        battery_pose.push_back(t.transform.translation.x);
                        battery_pose.push_back(t.transform.translation.y);
                        battery_pose.push_back(t.transform.translation.z);
                        battery_pose.push_back(t.transform.rotation.x);
                        battery_pose.push_back(t.transform.rotation.y);
                        battery_pose.push_back(t.transform.rotation.z);
                        battery_pose.push_back(t.transform.rotation.w);
                        abc = 2;
                    }

                    if (colorLength == 3 && abc == 2)
                    {
                        battery_pose.push_back(t.transform.translation.x);
                        battery_pose.push_back(t.transform.translation.y);
                        battery_pose.push_back(t.transform.translation.z);
                        battery_pose.push_back(t.transform.rotation.x);
                        battery_pose.push_back(t.transform.rotation.y);
                        battery_pose.push_back(t.transform.rotation.z);
                        battery_pose.push_back(t.transform.rotation.w);
                    }
                }
            }
        }

    void battery_data(int part_color, int part_type, std::string &bat_color, std::string &bat_type)
    {
        // Assign color
        switch (part_color)
        {
        case mage_msgs::msg::Part::BLUE:
            bat_color = "BLUE";
            break;
        case mage_msgs::msg::Part::GREEN:
            bat_color = "GREEN";
            break;
        case mage_msgs::msg::Part::ORANGE:
            bat_color = "ORANGE";
            break;
        case mage_msgs::msg::Part::RED:
            bat_color = "RED";
            break;
        case mage_msgs::msg::Part::PURPLE:
            bat_color = "PURPLE";
            break;
        }

        // Assign type
        switch (part_type)
        {
        case mage_msgs::msg::Part::BATTERY:
            bat_type = "BATTERY";
            break;
        case mage_msgs::msg::Part::PUMP:
            bat_type = "PUMP";
            break;
        case mage_msgs::msg::Part::SENSOR:
            bat_type = "SENSOR";
            break;
        case mage_msgs::msg::Part::REGULATOR:
            bat_type = "REGULATOR";
            break;
        }
    }

    void camera_image_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
    {
        for (const auto &part_pose : msg->part_poses)
        {
            int part_color = part_pose.part.color;
            int part_type = part_pose.part.type;

            std::string bat_color, bat_type;
            battery_data(part_color, part_type, bat_color, bat_type);


            // Check for uniqueness and store
            BatteryData data;
            data.color = bat_color;
            data.type = bat_type;

            if (unique_batteries.insert(data).second)
            {
                RCLCPP_INFO(this->get_logger(), "Added new battery with color: %s and type: %s", bat_color.c_str(), bat_type.c_str());
            }
        }
    }
    rclcpp::TimerBase::SharedPtr timer_a_{nullptr};
    // rclcpp::TimerBase::SharedPtr timer_b_{nullptr};
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_{nullptr};
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_a_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_a_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_b_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_b_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr subscriptionc_;
    // std::string target_frame_;
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr subscription_;
    std::set<BatteryData> unique_batteries;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameListener>());
    rclcpp::shutdown();
    return 0;
}