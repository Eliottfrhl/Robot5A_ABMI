#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "interfaces/msg/aruco_marker.hpp"
#include "interfaces/msg/aruco_markers.hpp"
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <sstream>
#include <fstream>

struct CameraInfo {
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
};

class PoseTransformer : public rclcpp::Node
{
public:
    PoseTransformer() : Node("pose_transformer")
    {
        poses_subscription_ = this->create_subscription<interfaces::msg::ArucoMarkers>(
            "/aruco_markers", 10, std::bind(&PoseTransformer::poses_callback, this, std::placeholders::_1));
        
        load_camera_info("src/robot_pose_transformer/config/distances.yaml");
    }

private:
    void poses_callback(const interfaces::msg::ArucoMarkers::SharedPtr msg)
    {
        Eigen::Matrix4d camera_transform = get_camera_transform_matrix();
        Eigen::Matrix4d camera_transform_inv = camera_transform.inverse();

        for (const auto &marker : msg->markers)
        {
            Eigen::Matrix4d marker_transform = get_transformation_matrix(marker.position, marker.orientation);
            Eigen::Matrix4d marker_in_world = camera_transform_inv * marker_transform;

            //std::stringstream ss;
            ///ss << "Transformation Matrix for marker ID " << marker.id << ":\n" 
            //   << marker_in_world.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ";\n", "[", "]", "[", "]"));
            //RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());

            // Convert position to centimeters
            Eigen::Vector3d position_cm = marker_in_world.block<3, 1>(0, 3) * 100.0;

            // Convert orientation to degrees
            Eigen::Matrix3d rotation_matrix = marker_in_world.block<3, 3>(0, 0);
            Eigen::Quaterniond orientation_deg = Eigen::Quaterniond(rotation_matrix);
            orientation_deg.normalize();
            Eigen::Vector3d euler_angles = orientation_deg.toRotationMatrix().eulerAngles(0, 1, 2) * 180.0 / M_PI;

            RCLCPP_INFO(this->get_logger(), "Marker ID: %d", marker.id);
            RCLCPP_INFO(this->get_logger(), "Position (cm): x=%.2f, y=%.2f, z=%.2f", position_cm.x(), position_cm.y(), position_cm.z());
            RCLCPP_INFO(this->get_logger(), "Orientation (degrees): roll=%.2f, pitch=%.2f, yaw=%.2f", euler_angles.x(), euler_angles.y(), euler_angles.z());
        }
    }

    Eigen::Matrix4d get_transformation_matrix(const geometry_msgs::msg::Point &position, const geometry_msgs::msg::Quaternion &orientation)
    {
        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

        // Translation
        transformation_matrix(0, 3) = position.x;
        transformation_matrix(1, 3) = position.y;
        transformation_matrix(2, 3) = position.z;

        // Rotation
        tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
        tf2::Matrix3x3 m(q);
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                transformation_matrix(i, j) = m[i][j];
            }
        }

        return transformation_matrix;
    }

    void load_camera_info(const std::string &file_path)
    {
        YAML::Node config = YAML::LoadFile(file_path);

        const auto &camera_config = config["cameras"][0];
        camera_info_.position = Eigen::Vector3d(camera_config["distance_to_origin"]["x"].as<double>(),
                                                camera_config["distance_to_origin"]["y"].as<double>(),
                                                camera_config["distance_to_origin"]["z"].as<double>());

        // Assuming the angles are in degrees and need to be converted to radians
        double roll = camera_config["euler_angles"]["x"].as<double>() * M_PI / 180.0;
        double pitch = camera_config["euler_angles"]["y"].as<double>() * M_PI / 180.0;
        double yaw = camera_config["euler_angles"]["z"].as<double>() * M_PI / 180.0;

        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        camera_info_.orientation = Eigen::Quaterniond(q.w(), q.x(), q.y(), q.z());
    }

    Eigen::Matrix4d get_camera_transform_matrix()
    {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

        // Translation
        transform(0, 3) = camera_info_.position.x();
        transform(1, 3) = camera_info_.position.y();
        transform(2, 3) = camera_info_.position.z();

        // Rotation
        Eigen::Matrix3d rotation_matrix = camera_info_.orientation.toRotationMatrix();
        transform.block<3, 3>(0, 0) = rotation_matrix;

        return transform;
    }

    rclcpp::Subscription<interfaces::msg::ArucoMarkers>::SharedPtr poses_subscription_;
    CameraInfo camera_info_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseTransformer>());
    rclcpp::shutdown();
    return 0;
}
