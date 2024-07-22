#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>

// Définition de la classe CameraCalibrator qui hérite de rclcpp::Node
class CameraCalibrator : public rclcpp::Node {
public:
    // Constructeur de la classe
    CameraCalibrator() : Node("camera_calibrator"), detected_chessboard_count_(0) {
        // Souscription au topic d'image
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "ue_grayscale_image", 10, std::bind(&CameraCalibrator::imageCallback, this, std::placeholders::_1));

        // Initialisation des points objets pour le chessboard
        objp_.resize(num_corners_x * num_corners_y);
        for (int i = 0; i < num_corners_y; i++) {
            for (int j = 0; j < num_corners_x; j++) {
                objp_[i * num_corners_x + j] = cv::Point3f(j, i, 0);
            }
        }
    }

private:
    // Callback pour la réception des images
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        std::cout << "Received image" << std::endl;
        cv::Mat frame;
        try {
            // Conversion du message ROS en image OpenCV
            frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Redimensionnement de l'image pour accélérer le traitement
        cv::resize(frame, frame, cv::Size(960, 540));

        // Détection des coins du chessboard
        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(frame, cv::Size(num_corners_x, num_corners_y), corners);

        if (found) {
            RCLCPP_INFO(this->get_logger(), "Chessboard corners found!");
            detected_chessboard_count_++;
            obj_points_.push_back(objp_);
            img_points_.push_back(corners);

            // Affichage des coins détectés
            cv::drawChessboardCorners(frame, cv::Size(num_corners_x, num_corners_y), corners, found);
            cv::imshow("Chessboard Corners", frame);
            cv::waitKey(500);  // Attendre 500 ms pour visualiser les coins détectés
        }

        // Si le nombre de chessboards détectés est suffisant, lancer la calibration
        if (detected_chessboard_count_ > 50) {
            calibrateCamera();
        }
    }

    // Fonction pour calibrer la caméra
    void calibrateCamera() {
        cv::Mat camMatrix, distCoeffs;
        std::vector<cv::Mat> rvecs, tvecs;

        // Calibration de la caméra
        cv::calibrateCamera(obj_points_, img_points_, cv::Size(img_width_, img_height_), camMatrix, distCoeffs, rvecs, tvecs);

        // Affichage de la matrice de la caméra et des coefficients de distorsion
        std::cout << "Camera Matrix:" << std::endl << camMatrix << std::endl;
        std::cout << "Distortion Coefficients:" << std::endl << distCoeffs << std::endl;

        // Sauvegarde des résultats de la calibration dans un fichier YAML
        cv::FileStorage fs("src/comm_ue/config/camera_calibration.yaml", cv::FileStorage::WRITE);
        fs << "camera_matrix" << camMatrix;
        fs << "distortion_coefficients" << distCoeffs;
        fs.release();

        RCLCPP_INFO(this->get_logger(), "Camera calibration completed and saved to camera_calibration.yaml");

        rclcpp::shutdown();  // Arrêter le nœud après la calibration
    }

    // Déclaration des variables membres
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;  // Souscription au topic d'images
    std::vector<std::vector<cv::Point3f>> obj_points_;  // Points objets 3D
    std::vector<std::vector<cv::Point2f>> img_points_;  // Points images 2D
    std::vector<cv::Point3f> objp_;  // Points objets pour un chessboard
    int detected_chessboard_count_;  // Compteur de chessboards détectés
    const int num_corners_x = 9;  // Nombre de coins en X du chessboard
    const int num_corners_y = 6;  // Nombre de coins en Y du chessboard
    int img_width_ = 1920;  // Largeur de l'image
    int img_height_ = 1080;  // Hauteur de l'image
};

// Fonction principale
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);  // Initialisation de ROS2
    auto node = std::make_shared<CameraCalibrator>();  // Création du nœud CameraCalibrator
    rclcpp::spin(node);  // Exécution du nœud
    rclcpp::shutdown();  // Arrêt de ROS2
    return 0;
}
