#!/usr/bin/env python3

from os import getpid

import rclpy
from rclpy.node import Node

from ros2_qos_tools import qos_lookup


class QosRelay(Node):
    def __init__(self):
        super().__init__("qos_relay_{}".format(getpid()))

        self.declare_parameter("inbound_topic", "")
        inbound_topic = self.get_parameter("inbound_topic").get_parameter_value().string_value
        inbound_topic_info = qos_lookup.get_topic_infos(self, qos_lookup.PUB, inbound_topic)[0]
        self.declare_parameter("outbound_topic", "")
        outbound_topic = self.get_parameter("outbound_topic").get_parameter_value().string_value
        outbound_topic_info = qos_lookup.get_topic_infos(self, qos_lookup.SUB, outbound_topic)[0]

        self.relay_ok_ = inbound_topic_info.topic_type == outbound_topic_info.topic_type
        if self.relay_ok_:
            msg_type = qos_lookup.get_msg_type_from_str(inbound_topic_info.topic_type)
            outbound_qos_profile = qos_lookup.fix_qos_profile(outbound_topic_info.qos_profile)
            inbound_qos_profile = qos_lookup.fix_qos_profile(inbound_topic_info.qos_profile)
            self.publisher_ = self.create_publisher(msg_type, outbound_topic, outbound_qos_profile)
            self.subscriber_ = self.create_subscription(
                msg_type, inbound_topic, lambda msg: self.publisher_.publish(msg), inbound_qos_profile
            )
            self.get_logger().info(
                "{}: {} -> {} relay initialized".format(inbound_topic_info.topic_type, inbound_topic, outbound_topic)
            )
        else:
            self.get_logger().error(
                "Type mismatch: {} and {}".format(inbound_topic_info.topic_type, outbound_topic_info.topic_type)
            )

    @property
    def relay_ok(self):
        return self.relay_ok_


def main(args=None):
    rclpy.init(args=args)
    node = QosRelay()
    if node.relay_ok:
        rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
