#include "rclcpp/rclcpp.hpp"
#include <sys/ipc.h>
#include <sys/shm.h>
#include <cstring>
#include <iostream>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <errno.h> // Pour errno

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("ue_image_receiver");

    char* data;
    key_t key = ftok("/home/eliott-frohly/Documents/Unreal Projects/ROS2UEConnection/Source/ROS2UEConnection/dataImage.conf", 1);
    if (key == -1)
    {
        RCLCPP_ERROR(node->get_logger(), "Error in ftok: %s", strerror(errno));
        return 1;
    }

    // Taille ajustée pour une image 1920x1080 en niveaux de gris, alignée sur 4096 octets
    int imageWidth = 1920;
    int imageHeight = 1080;
    int channels = 1; // Grayscale
    int shmSize = imageWidth * imageHeight * channels;
    int pageSize = sysconf(_SC_PAGESIZE);
    shmSize = ((shmSize + pageSize - 1) / pageSize) * pageSize; // Alignement sur la taille de page

    RCLCPP_INFO(node->get_logger(), "Requested shared memory size: %d bytes", shmSize);
    RCLCPP_INFO(node->get_logger(), "Page size: %d bytes", pageSize);

    // Vérifier si le segment de mémoire partagée existe déjà
    int shmid = shmget(key, 0, 0666);
    if (shmid != -1)
    {
        // Le segment existe, le supprimer
        RCLCPP_INFO(node->get_logger(), "Shared memory segment already exists. Deleting it.");

        // Détacher le segment existant
        data = (char*)shmat(shmid, NULL, 0);
        if (data != (void*)-1)
        {
            if (shmdt(data) == -1)
            {
                RCLCPP_ERROR(node->get_logger(), "Error in shmdt: %s", strerror(errno));
                return 1;
            }
        }

        // Supprimer le segment existant
        if (shmctl(shmid, IPC_RMID, NULL) == -1)
        {
            RCLCPP_ERROR(node->get_logger(), "Error in shmctl (IPC_RMID): %s", strerror(errno));
            return 1;
        }

        RCLCPP_INFO(node->get_logger(), "Deleted existing shared memory segment.");
    }

    // Créer un nouveau segment avec la taille correcte
    shmid = shmget(key, shmSize, 0666 | IPC_CREAT | IPC_EXCL);
    if (shmid == -1)
    {
        RCLCPP_ERROR(node->get_logger(), "Error in shmget: %s", strerror(errno));
        return 1;
    }

    // Attacher le nouveau segment
    data = (char*)shmat(shmid, NULL, 0);
    if (data == (void*)-1)
    {
        RCLCPP_ERROR(node->get_logger(), "Error in shmat: %s", strerror(errno));
        return 1;
    }

    // Create the publisher
    auto image_publisher = node->create_publisher<sensor_msgs::msg::Image>("ue_grayscale_image", 10);

    RCLCPP_INFO(node->get_logger(), "Start Receiving using key %d and shmid %d..", key, shmid);
    while (rclcpp::ok())
    {
        if (shmid != -1 && data != (void*)-1)
        {
            // Copie des données de la mémoire partagée dans une matrice OpenCV
            cv::Mat image(imageHeight, imageWidth, CV_8UC1, data);

            // Convertir l'image en message ROS2
            std_msgs::msg::Header header;
            header.stamp = node->now();
            header.frame_id = "ue_camera_frame";

            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(header, "mono8", image).toImageMsg();

            // Publier l'image
            image_publisher->publish(*msg);

            // Redimensionner l'image pour l'affichage
            cv::resize(image, image, cv::Size(960, 540));

            // Afficher l'image pour vérification (optionnel)
            cv::imshow("Received Grayscale Image", image);
            cv::waitKey(1); // Met à jour l'affichage et gère les événements de la fenêtre
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (shmdt(data) == -1)
    {
        RCLCPP_ERROR(node->get_logger(), "Error in shmdt: %s", strerror(errno));
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "RELEASED SHARED MEMORY SEGMENT");
    return 0;
}
