#ifndef ROS2_QOS_TOOLS__QOS_LOOKUP_HPP
#define ROS2_QOS_TOOLS__QOS_LOOKUP_HPP

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

namespace ros2_qos_tools
{
    static rclcpp::QoS lookup_qos(
        const rclcpp::Node &node,
        const std::string &topic,
        const rclcpp::EndpointType endpoint_type,
        const float timeout = -1.0)
    {
        auto start_time = std::chrono::system_clock::now();
        while (rclcpp::ok())
        {
            std::vector<rclcpp::TopicEndpointInfo> endpoint_infos;
            switch (endpoint_type)
            {
            case rclcpp::EndpointType::Publisher:
                endpoint_infos = node.get_publishers_info_by_topic(topic);
                break;
            case rclcpp::EndpointType::Subscription:
                endpoint_infos = node.get_subscriptions_info_by_topic(topic);
                break;
            default:
                RCLCPP_ERROR(node.get_logger(), "QoS lookup [%s]: invalid endpoint type requested", topic.c_str());
                return rclcpp::SystemDefaultsQoS();
            }

            if (!endpoint_infos.empty())
            {
                auto qos = endpoint_infos[0].qos_profile();

                if (qos.history() == rclcpp::HistoryPolicy::Unknown)
                {
                    qos.history(rclcpp::HistoryPolicy::SystemDefault);
                }

                return qos;
            }

            std::chrono::duration<float> dt = std::chrono::system_clock::now() - start_time;
            if (timeout > 0.0 && dt.count() > timeout)
            {
                RCLCPP_ERROR(node.get_logger(), "QoS lookup [%s]: timeout", topic.c_str());
                return rclcpp::SystemDefaultsQoS();
            }

            RCLCPP_WARN(node.get_logger(), "QoS lookup [%s]: no endpoints available", topic.c_str());
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        return rclcpp::SystemDefaultsQoS();
    }
}

#endif