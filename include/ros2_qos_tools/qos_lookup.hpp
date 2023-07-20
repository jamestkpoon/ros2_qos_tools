#ifndef ROS2_QOS_TOOLS_QOS_LOOKUP_HPP
#define ROS2_QOS_TOOLS_QOS_LOOKUP_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

namespace ros2_qos_tools
{
    inline rclcpp::QoS get_sub_qos_from_topic(const rclcpp::Node &node, const std::string &topic)
    {
        const auto endpoint_info = node.get_subscriptions_info_by_topic(topic);
        if (endpoint_info.empty())
        {
            RCLCPP_WARN(node.get_logger(), "No subscribers for %s", topic.c_str());
            return rclcpp::SystemDefaultsQoS();
        }
        else
        {
            return endpoint_info[0].qos_profile();
        }
    }

    inline rclcpp::QoS get_pub_qos_from_topic(const rclcpp::Node &node, const std::string &topic)
    {
        const auto endpoint_info = node.get_publishers_info_by_topic(topic);
        if (endpoint_info.empty())
        {
            RCLCPP_WARN(node.get_logger(), "No subscribers for %s", topic.c_str());
            return rclcpp::SystemDefaultsQoS();
        }
        else
        {
            return endpoint_info[0].qos_profile();
        }
    }
}

#endif