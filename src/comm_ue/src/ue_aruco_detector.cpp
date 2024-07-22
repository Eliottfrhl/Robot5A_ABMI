#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "interfaces/msg/aruco_marker.hpp"
#include "interfaces/msg/aruco_markers.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include <Eigen/Dense>
#include <unordered_map>

// Définition de la classe ArucoDetector qui hérite de rclcpp::Node
class ArucoDetector : public rclcpp::Node {
public:
    // Constructeur de la classe ArucoDetector
    ArucoDetector() : Node("aruco_detector"), tf_broadcaster_(this) {
        // Création de la subscription pour recevoir les images
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "ue_grayscale_image", 10, std::bind(&ArucoDetector::imageCallback, this, std::placeholders::_1));
        
        // Lire les paramètres de calibration de la caméra
        readCameraCalibration("src/comm_ue/config/camera_calibration.yaml", camMatrix_, distCoeffs_);
        
        // Lire le fichier de transformations
        readTransforms("src/comm_ue/config/transform.yaml");
    }

private:
    // Fonction pour lire la calibration de la caméra à partir d'un fichier
    void readCameraCalibration(const std::string& filename, cv::Mat& camMatrix, cv::Mat& distCoeffs) {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera calibration file: %s", filename.c_str());
            return;
        }

        fs["camera_matrix"] >> camMatrix;
        fs["distortion_coefficients"] >> distCoeffs;

        fs.release();
    }

    // Fonction pour lire les transformations à partir d'un fichier
    void readTransforms(const std::string& filename) {
        YAML::Node config = YAML::LoadFile(filename);
        if (config["camera"]) {
            camera_transform_ = parseTransform(config["camera"][0]["transform"]);
        }
        for (const auto& solid_node : config["solids"]) {
            int solid_id = solid_node["id"].as<int>();
            for (const auto& aruco_node : solid_node["aruco_markers"]) {
                int aruco_id = aruco_node["id"].as<int>();
                Eigen::Matrix4d transform = parseTransform(aruco_node["transform"]);
                solid_to_aruco_transforms_[aruco_id] = {solid_id, transform};
            }
        }
    }

    // Fonction pour parser une transformation à partir d'un nœud YAML
    Eigen::Matrix4d parseTransform(const YAML::Node& node) {
        Eigen::Matrix4d transform;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                transform(i, j) = node[i][j].as<double>();
            }
        }
        return transform;
    }

    // Callback pour le traitement des images reçues
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame;
        try {
            // Conversion du message ROS en image OpenCV
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Redimensionnement de l'image
        cv::Size newSize;
        newSize.height = 640;
        newSize.width = frame.cols * newSize.height / frame.rows;
        cv::resize(frame, frame, newSize);

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
        cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::aruco::ArucoDetector detector(dictionary, detectorParams);
        detector.detectMarkers(frame, markerCorners, markerIds, rejectedCandidates);

        float markerLength = 1; // Longueur du marqueur en unités arbitraires

        cv::Mat objPoints(4, 1, CV_32FC3);
        objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
        objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);

        size_t nMarkers = markerCorners.size();
        std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

        std::unordered_map<int, std::vector<Eigen::Matrix4d>> solid_transforms;

        // Dessiner les marqueurs détectés et les axes
        cv::Mat outputImage = frame.clone();
        if (!markerIds.empty()) {
            for (size_t i = 0; i < nMarkers; i++) {
                // Calcul de la position et de l'orientation des marqueurs
                cv::solvePnP(objPoints, markerCorners.at(i), camMatrix_, distCoeffs_, rvecs.at(i), tvecs.at(i));
                cv::drawFrameAxes(outputImage, camMatrix_, distCoeffs_, rvecs.at(i), tvecs.at(i), markerLength * 1.5f, 2);

                cv::Mat rotation_matrix;
                cv::Rodrigues(rvecs.at(i), rotation_matrix);

                Eigen::Matrix4d camera_to_aruco = Eigen::Matrix4d::Identity();
                for (int row = 0; row < 3; ++row) {
                    for (int col = 0; col < 3; ++col) {
                        camera_to_aruco(row, col) = rotation_matrix.at<double>(row, col);
                    }
                    camera_to_aruco(row, 3) = tvecs[i][row];
                }

                // Vérifier si le marqueur détecté est dans la map
                if (solid_to_aruco_transforms_.find(markerIds[i]) != solid_to_aruco_transforms_.end()) {
                    auto [solid_id, solid_to_aruco] = solid_to_aruco_transforms_[markerIds[i]];
                    Eigen::Matrix4d camera_to_solid = camera_to_aruco * solid_to_aruco.inverse();
                    solid_transforms[solid_id].push_back(camera_to_solid);
                }
            }

            cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);

            for (const auto& [solid_id, transforms] : solid_transforms) {
                if (transforms.empty()) continue;

                // Calcul de la transformation moyenne du solide dans le repère de la caméra
                Eigen::Matrix4d average_transform = Eigen::Matrix4d::Zero();
                for (const auto& transform : transforms) {
                    average_transform += transform;
                }
                average_transform /= transforms.size();

                // Calcul de la transformation du solide dans le repère fixe
                Eigen::Matrix4d fixed_to_camera = camera_transform_;
                Eigen::Matrix4d fixed_to_solid = fixed_to_camera * average_transform;

                // Préparation du message TransformStamped
                geometry_msgs::msg::TransformStamped transformStamped;
                transformStamped.header.stamp = this->now();
                transformStamped.header.frame_id = "fixed_frame";
                transformStamped.child_frame_id = "solid_" + std::to_string(solid_id);
                transformStamped.transform.translation.x = fixed_to_solid(0, 3);
                transformStamped.transform.translation.y = fixed_to_solid(1, 3);
                transformStamped.transform.translation.z = fixed_to_solid(2, 3);

                Eigen::Matrix3d rotation = fixed_to_solid.block<3, 3>(0, 0);
                Eigen::Quaterniond quaternion(rotation);
                transformStamped.transform.rotation.x = quaternion.x();
                transformStamped.transform.rotation.y = quaternion.y();
                transformStamped.transform.rotation.z = quaternion.z();
                transformStamped.transform.rotation.w = quaternion.w();

                // Envoi de la transformation
                tf_broadcaster_.sendTransform(transformStamped);
            }
        }

        // Afficher l'image pour vérification (optionnel)
        cv::imshow("Detected ArUco markers", outputImage);
        cv::waitKey(1);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_; // Subscription pour les images
    cv::Mat camMatrix_, distCoeffs_; // Matrices de calibration de la caméra
    tf2_ros::TransformBroadcaster tf_broadcaster_; // Broadcaster pour les transformations
    std::unordered_map<int, std::pair<int, Eigen::Matrix4d>> solid_to_aruco_transforms_; // Map des transformations ArUco
    Eigen::Matrix4d camera_transform_; // Transformation de la caméra
};

// Fonction principale
int main(int argc, char **argv) {
    rclcpp::init(argc, argv); // Initialisation de ROS2
    auto node = std::make_shared<ArucoDetector>(); // Création du nœud
    rclcpp::spin(node); // Exécution du nœud
    rclcpp::shutdown(); // Arrêt de ROS2
    return 0;
}
