#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <sys/ipc.h>
#include <sys/shm.h>
#include <cstring>
#include <iostream>
#include <thread>
#include <chrono>
#include <errno.h>

// Définition de la classe UEJointsCommand qui hérite de rclcpp::Node
class UEJointsCommand : public rclcpp::Node
{
public:
    // Constructeur de la classe UEJointsCommand
    UEJointsCommand() : Node("ue_joints_command")
    {
        // Création d'une subscription au topic "ue_values"
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "ue_values", 10, std::bind(&UEJointsCommand::topic_callback, this, std::placeholders::_1));

        // Génération de la clé pour la mémoire partagée
        key_t key = ftok("/home/eliott-frohly/Documents/Unreal Projects/ROS2UEConnection/Source/ROS2UEConnection/dataCommand.conf", 1);
        if (key == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in ftok: %s", strerror(errno));
            rclcpp::shutdown();
            return;
        }

        // Calcul de la taille de la mémoire partagée nécessaire
        shmSize = 5 * sizeof(double);
        int pageSize = sysconf(_SC_PAGESIZE);
        shmSize = ((shmSize + pageSize - 1) / pageSize) * pageSize; // Alignement sur la taille de page

        RCLCPP_INFO(this->get_logger(), "Requested shared memory size: %d bytes", shmSize);
        RCLCPP_INFO(this->get_logger(), "Page size: %d bytes", pageSize);

        // Création ou accès à un segment de mémoire partagée
        shmid = shmget(key, shmSize, 0666 | IPC_CREAT | IPC_EXCL);
        if (shmid == -1)
        {
            if (errno == EEXIST)
            {
                RCLCPP_INFO(this->get_logger(), "Shared memory segment already exists. Trying to access it with key: %d", key);

                // Tentative d'accès au segment existant
                shmid = shmget(key, 0, 0666);
                if (shmid == -1)
                {
                    RCLCPP_ERROR(this->get_logger(), "Error in shmget for existing segment with key %d: %s", key, strerror(errno));
                    rclcpp::shutdown();
                    return;
                }
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Error in shmget with key %d: %s", key, strerror(errno));
                rclcpp::shutdown();
                return;
            }
        }

        // Attacher le segment de mémoire partagée au processus
        data = (double*)shmat(shmid, NULL, 0);
        if (data == (void*)-1)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in shmat with key %d: %s", key, strerror(errno));
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully created/accessed shared memory segment with key: %d and shmid: %d", key, shmid);
    }

    // Destructeur de la classe UEJointsCommand
    ~UEJointsCommand()
    {
        // Détachement du segment de mémoire partagée
        if (shmdt(data) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in shmdt: %s", strerror(errno));
        }

        RCLCPP_INFO(this->get_logger(), "RELEASED SHARED MEMORY SEGMENT");
    }

private:
    // Callback appelé lorsque des messages sont reçus sur le topic "ue_values"
    void topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // Vérifier que le message contient au moins 5 valeurs
        if (msg->data.size() >= 5)
        {
            // Copier les valeurs dans la mémoire partagée
            std::memcpy(data, msg->data.data(), 5 * sizeof(double));
            RCLCPP_INFO(this->get_logger(), "Updated shared memory with new values");
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_; // Subscription ROS2
    double* data; // Pointeur vers la mémoire partagée
    int shmid; // ID du segment de mémoire partagée
    int shmSize; // Taille du segment de mémoire partagée
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv); // Initialisation de ROS2
    auto node = std::make_shared<UEJointsCommand>(); // Création du nœud
    rclcpp::spin(node); // Exécution du nœud
    rclcpp::shutdown(); // Arrêt de ROS2
    return 0;
}
