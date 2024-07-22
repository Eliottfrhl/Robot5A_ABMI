#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/aruco_marker.hpp"
#include "interfaces/msg/aruco_markers.hpp"

class ArucoSubscriber : public rclcpp::Node {
public:
    ArucoSubscriber() : Node("aruco_subscriber") {
        marker_subscription_ = this->create_subscription<interfaces::msg::ArucoMarkers>(
            "aruco_markers", 10, std::bind(&ArucoSubscriber::markerCallback, this, std::placeholders::_1));
    }

private:
    void markerCallback(const interfaces::msg::ArucoMarkers::SharedPtr msg) {
        for (const auto &marker : msg->markers) {
            RCLCPP_INFO(this->get_logger(), "Detected ArUco marker ID: %d", marker.id);
            RCLCPP_INFO(this->get_logger(), "Position - x: %f, y: %f, z: %f", marker.position.x, marker.position.y, marker.position.z);
            RCLCPP_INFO(this->get_logger(), "Orientation - x: %f, y: %f, z: %f, w: %f", marker.orientation.x, marker.orientation.y, marker.orientation.z, marker.orientation.w);
        }
    }

    rclcpp::Subscription<interfaces::msg::ArucoMarkers>::SharedPtr marker_subscription_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
