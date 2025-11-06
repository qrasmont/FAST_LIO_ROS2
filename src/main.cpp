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
        node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
        node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

        std::string bag_file;
        node->get_parameter("bag_file", bag_file);

        if (bag_file.empty())
        {
            RCLCPP_INFO(node->get_logger(), "Starting in LIVE mode.");

            rclcpp::executors::SingleThreadedExecutor exec;
            exec.add_node(node->get_node_base_interface());
            exec.spin();
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
