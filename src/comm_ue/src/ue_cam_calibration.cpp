#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

class CameraCalibrator : public rclcpp::Node {
public:
    CameraCalibrator() : Node("camera_calibrator"), detected_chessboard_count_(0) {
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "ue_grayscale_image", 10, std::bind(&CameraCalibrator::imageCallback, this, std::placeholders::_1));

        // Initialise les points objets pour le chessboard
        objp_.resize(num_corners_x * num_corners_y);
        for (int i = 0; i < num_corners_y; i++) {
            for (int j = 0; j < num_corners_x; j++) {
                objp_[i * num_corners_x + j] = cv::Point3f(j, i, 0);
            }
        }
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        std::cout << "Received image" << std::endl;
        cv::Mat frame;
        try {
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Redimensionner l'image
        cv::resize(frame, frame, cv::Size(960, 540));

        // Détecter les coins du chessboard
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(frame, cv::Size(num_corners_x, num_corners_y), corners);

        if (found) {
            RCLCPP_INFO(this->get_logger(), "Chessboard corners found!");
            detected_chessboard_count_++;
            obj_points_.push_back(objp_);
            img_points_.push_back(corners);

            // Afficher les coins détectés
            cv::drawChessboardCorners(frame, cv::Size(num_corners_x, num_corners_y), corners, found);
            cv::imshow("Chessboard Corners", frame);
            cv::waitKey(500);
        }

        if (detected_chessboard_count_ > 50) {  // Nombre minimum de vues pour la calibration
            calibrateCamera();
        }
    }

    void calibrateCamera() {
        cv::Mat camMatrix, distCoeffs;
        std::vector<cv::Mat> rvecs, tvecs;

        cv::calibrateCamera(obj_points_, img_points_, cv::Size(img_width_, img_height_), camMatrix, distCoeffs, rvecs, tvecs);

        // Afficher la matrice de la caméra et les coefficients de distorsion
        std::cout << "Camera Matrix:" << std::endl << camMatrix << std::endl;
        std::cout << "Distortion Coefficients:" << std::endl << distCoeffs << std::endl;

        // Sauvegarder les résultats de la calibration dans un fichier YAML
        cv::FileStorage fs("src/comm_ue/config/camera_calibration.yaml", cv::FileStorage::WRITE);
        fs << "camera_matrix" << camMatrix;
        fs << "distortion_coefficients" << distCoeffs;
        fs.release();

        RCLCPP_INFO(this->get_logger(), "Camera calibration completed and saved to camera_calibration.yaml");

        rclcpp::shutdown();  // Arrêter le nœud après la calibration
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    std::vector<std::vector<cv::Point3f>> obj_points_;
    std::vector<std::vector<cv::Point2f>> img_points_;
    std::vector<cv::Point3f> objp_;
    int detected_chessboard_count_;
    const int num_corners_x = 9;
    const int num_corners_y = 6;
    int img_width_ = 1920;
    int img_height_ = 1080;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraCalibrator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
