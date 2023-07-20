#ifndef ROS2_QOS_TOOLS_QOS_LOOKUP_HPP
#define ROS2_QOS_TOOLS_QOS_LOOKUP_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

namespace ros2_qos_tools
{
    inline rclcpp::QoS get_qos_from_endpoint_infos(const std::vector<rclcpp::TopicEndpointInfo> &endpoint_infos,
                                                   const rclcpp::Logger &logger, const std::string &topic)
    {
        if (endpoint_infos.empty())
        {
            RCLCPP_WARN(logger, "No endpoints for %s", topic.c_str());
            return rclcpp::SystemDefaultsQoS();
        }
        else
        {
            auto qos = endpoint_infos[0].qos_profile();

            if (qos.history() == rclcpp::HistoryPolicy::Unknown)
            {
                qos.history(rclcpp::HistoryPolicy::SystemDefault);
            }

            return qos;
        }
    }

    inline rclcpp::QoS get_sub_qos_from_topic(const rclcpp::Node &node, const std::string &topic)
    {
        return get_qos_from_endpoint_infos(node.get_subscriptions_info_by_topic(topic), node.get_logger(), topic);
    }

    inline rclcpp::QoS get_pub_qos_from_topic(const rclcpp::Node &node, const std::string &topic)
    {
        return get_qos_from_endpoint_infos(node.get_publishers_info_by_topic(topic), node.get_logger(), topic);
    }
}

#endif