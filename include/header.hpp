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
class FrameListener : public rclcpp::Node
{
public:
private:
    double current_yaw_;
    int desc = 0;
    int key = 0;
    double start_yaw_;
    double target_yaw_;
    bool pose_key_;
    vector <float> battery_pose;
    int check;
    
    /**
     * @brief the acuro marker
     * 
     * @param msg 
     */
    void aruco_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);
    /**
     * @brief move the robot
     * 
     * @param action 
     */
    void perform_action(const std::string &action);
    /**
     * @brief get the odometry data
     * 
     * @param msg 
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    /**
     * @brief control the robot euler angles
     * 
     * @param q 
     * @param roll 
     * @param pitch 
     * @param yaw 
     */
    void quaternionToEuler(const geometry_msgs::msg::Quaternion &q, double &roll, double &pitch, double &yaw);
    /**
     * @brief get the pose of the robot
     * 
     * @param msg 
     */
    void handle_turtle_pose(const std::shared_ptr<mage_msgs::msg::AdvancedLogicalCameraImage> msg);
    /**
     * @brief get the battery details
     * 
     * @param part_color 
     * @param part_type 
     * @param bat_color 
     * @param bat_type 
     */
    void battery_data(int part_color, int part_type, std::string &bat_color, std::string &bat_type);
    /**
     * @brief get the camera details
     * 
     * @param msg 
     */
    void camera_image_callback(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
};