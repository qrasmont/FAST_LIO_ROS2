#include "fast_lio/laserMapping.h"

void SigHandle(int sig)
{
    std::cout << "catch sig %d" << sig << std::endl;
    rclcpp::shutdown();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    signal(SIGINT, SigHandle);

    {
        auto node = std::make_shared<LaserMappingNode>();

        std::string bag_file;
        node->get_parameter("bag_file", bag_file);

        if (bag_file.empty())
        {
            RCLCPP_INFO(node->get_logger(), "Starting in LIVE mode.");
            rclcpp::spin(node);
        }
        else
        {
            RCLCPP_INFO(node->get_logger(), "Starting in OFFLINE mode for bag: %s", bag_file.c_str());
            node->process_bag_file(bag_file);
        }

    }

    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }

    return 0;
}
