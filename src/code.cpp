#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include "rosgraph_msgs/msg/clock.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
// #include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
// #include "turtlesim/msg/pose.hpp"
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"

class FramePublisher : public rclcpp::Node
{
public:
    FramePublisher()
        : Node("turtle_tf2_frame_publisher")
    {
        // Declare and acquire `turtlename` parameter
        // turtlename_ = this->declare_parameter<std::string>("turtlename", "turtle");

        // Initialize the transform broadcaster
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        // callback function on each message
        // std::ostringstream stream;
        // stream << "/" << turtlename_.c_str() << "/pose";
        // std::string topic_name = stream.str();

        subscriptiona_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            "aruco_markers", 10,
            std::bind(&FramePublisher::handle_arco_pose, this, std::placeholders::_1));
        subscriptionc_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
            "mage/advanced_logical_camera/image", rclcpp::SensorDataQoS(),
            std::bind(&FramePublisher::handle_objects_pose, this, std::placeholders::_1));
        subscriptionb_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", 10,
            std::bind(&FramePublisher::sub_clock_cb_, this, std::placeholders::_1));
        // subscription_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
        //     "aruco_markers", 10, std::bind(&MinimalPublisher::aruco_callback, this, std::placeholders::_1))
    }

private:
    void sub_clock_cb_(const rosgraph_msgs::msg::Clock::SharedPtr msg)
    {

        current_time_ = msg->clock;
    }
    void handle_arco_pose(const std::shared_ptr<ros2_aruco_interfaces::msg::ArucoMarkers> msg)
    {
        geometry_msgs::msg::TransformStamped t;


        // Read message content and assign it to
        // corresponding tf variables
        t.header.stamp = current_time_;
        t.header.frame_id = "camera_rgb_optical_frame";
        t.child_frame_id = "marker";

        // Turtle only exists in 2D, thus we get x and y translation
        // coordinates from the message and set the z coordinate to 0
        // const auto& pose = msg->poses;
        // aruco_pose_.poses[0].position.x;
        t.transform.translation.x = msg->poses[0].position.x;
        RCLCPP_INFO(this->get_logger(), "position.x from Arco marker: '%f'", msg->poses[0].position.x);
        t.transform.translation.y = msg->poses[0].position.y;
        RCLCPP_INFO(this->get_logger(), "position.y from Arco marker: '%f'", msg->poses[0].position.y);
        t.transform.translation.z = msg->poses[0].position.z;
        RCLCPP_INFO(this->get_logger(), "position.z from Arco marker: '%f'", msg->poses[0].position.z);

        // For the same reason, turtle can only rotate around one axis
        // and this why we set rotation in x and y to 0 and obtain
        // rotation in z axis from the message
        // tf2::Quaternion q;
        // q.setRPY(0, 0, msg->poses[1].orientation);
        t.transform.rotation.x = msg->poses[0].orientation.x;
        RCLCPP_INFO(this->get_logger(), "orientation.x from Arco marker: '%f'", msg->poses[0].orientation.x);
        t.transform.rotation.y = msg->poses[0].orientation.y;
        RCLCPP_INFO(this->get_logger(), "orientation.y from Arco marker: '%f'", msg->poses[0].orientation.y);
        t.transform.rotation.z = msg->poses[0].orientation.z;
        RCLCPP_INFO(this->get_logger(), "orientation.z from Arco marker: '%f'", msg->poses[0].orientation.z);
        t.transform.rotation.w = msg->poses[0].orientation.w;
        RCLCPP_INFO(this->get_logger(), "orientation.w from Arco marker: '%f'", msg->poses[0].orientation.w);

        // Send the transformation
        tf_broadcaster_->sendTransform(t);
        RCLCPP_INFO(this->get_logger(), "Arco frames published");
    }

    void handle_objects_pose(const std::shared_ptr<mage_msgs::msg::AdvancedLogicalCameraImage> msg)
    {
        if (msg->part_poses.size() != 0)
        {
            geometry_msgs::msg::TransformStamped t;

            // Read message content and assign it to
            // corresponding tf variables
            t.header.stamp = current_time_;
            t.header.frame_id = "logical_camera_link";
            t.child_frame_id = "object";

            // Turtle only exists in 2D, thus we get x and y translation
            // coordinates from the message and set the z coordinate to 0
            // const auto& pose = msg->poses;
            // aruco_pose_.poses[0].position.x;
            t.transform.translation.x = msg->part_poses[0].pose.position.x;
            RCLCPP_INFO(this->get_logger(), "position.x from Arco marker: '%f'", msg->part_poses[0].pose.position.x);
            t.transform.translation.y = msg->part_poses[0].pose.position.y;
            RCLCPP_INFO(this->get_logger(), "position.y from Arco marker: '%f'", msg->part_poses[0].pose.position.y);
            t.transform.translation.z = msg->part_poses[0].pose.position.z;
            RCLCPP_INFO(this->get_logger(), "position.z from Arco marker: '%f'", msg->part_poses[0].pose.position.z);

            // For the same reason, turtle can only rotate around one axis
            // and this why we set rotation in x and y to 0 and obtain
            // rotation in z axis from the message
            // tf2::Quaternion q;
            // q.setRPY(0, 0, msg->poses[1].orientation);
            t.transform.rotation.x = msg->part_poses[0].pose.orientation.x;
            RCLCPP_INFO(this->get_logger(), "orientation.x from Arco marker: '%f'", msg->part_poses[0].pose.orientation.x);
            t.transform.rotation.y = msg->part_poses[0].pose.orientation.y;
            RCLCPP_INFO(this->get_logger(), "orientation.y from Arco marker: '%f'", msg->part_poses[0].pose.orientation.y);
            t.transform.rotation.z = msg->part_poses[0].pose.orientation.z;
            RCLCPP_INFO(this->get_logger(), "orientation.z from Arco marker: '%f'", msg->part_poses[0].pose.orientation.z);
            t.transform.rotation.w = msg->part_poses[0].pose.orientation.w;
            RCLCPP_INFO(this->get_logger(), "orientation.w from Arco marker: '%f'", msg->part_poses[0].pose.orientation.w);

            // Send the transformation
            tf_broadcaster_->sendTransform(t);
            RCLCPP_INFO(this->get_logger(), "Objects frames published");
        }
    }
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr subscriptiona_;
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr subscriptionc_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr subscriptionb_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Time current_time_;
    // std::string turtlename_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FramePublisher>());
    rclcpp::shutdown();
    return 0;
}