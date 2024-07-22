#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <sys/ipc.h>
#include <sys/shm.h>
#include <cstring>
#include <iostream>
#include <thread>
#include <chrono>
#include <errno.h> // Pour errno

class UEJointsCommand : public rclcpp::Node
{
public:
    UEJointsCommand() : Node("ue_joints_command")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "ue_values", 10, std::bind(&UEJointsCommand::topic_callback, this, std::placeholders::_1));

        key_t key = ftok("/home/eliott-frohly/Documents/Unreal Projects/ROS2UEConnection/Source/ROS2UEConnection/dataCommand.conf", 1);
        if (key == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in ftok: %s", strerror(errno));
            rclcpp::shutdown();
            return;
        }

        shmSize = 5 * sizeof(double);
        int pageSize = sysconf(_SC_PAGESIZE);
        shmSize = ((shmSize + pageSize - 1) / pageSize) * pageSize; // Alignement sur la taille de page

        RCLCPP_INFO(this->get_logger(), "Requested shared memory size: %d bytes", shmSize);
        RCLCPP_INFO(this->get_logger(), "Page size: %d bytes", pageSize);

        shmid = shmget(key, shmSize, 0666 | IPC_CREAT | IPC_EXCL);
        if (shmid == -1)
        {
            if (errno == EEXIST)
            {
                RCLCPP_INFO(this->get_logger(), "Shared memory segment already exists. Trying to access it with key: %d", key);

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

        data = (double*)shmat(shmid, NULL, 0);
        if (data == (void*)-1)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in shmat with key %d: %s", key, strerror(errno));
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully created/accessed shared memory segment with key: %d and shmid: %d", key, shmid);
    }

    ~UEJointsCommand()
    {
        if (shmdt(data) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Error in shmdt: %s", strerror(errno));
        }

        RCLCPP_INFO(this->get_logger(), "RELEASED SHARED MEMORY SEGMENT");
    }

private:
    void topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() >= 5)
        {
            std::memcpy(data, msg->data.data(), 5 * sizeof(double));
            RCLCPP_INFO(this->get_logger(), "Updated shared memory with new values");
        }
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    double* data;
    int shmid;
    int shmSize;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UEJointsCommand>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}