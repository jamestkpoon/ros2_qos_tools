import pathlib
from importlib import import_module
from time import perf_counter, sleep

import rclpy
from rclpy import qos
from rclpy.node import Node
from rclpy.topic_endpoint_info import TopicEndpointTypeEnum

PUB = TopicEndpointTypeEnum.PUBLISHER
SUB = TopicEndpointTypeEnum.SUBSCRIPTION


def fix_qos_profile(qos_profile: qos.QoSProfile):
    if qos_profile.history == qos_profile.history.UNKNOWN:
        qos_profile.history = qos_profile.history.KEEP_LAST

    return qos_profile


def get_endpoint_qos(node: Node, endpoint_type: TopicEndpointTypeEnum, topic_name: str, timeout: float = -1.0):
    if (topic_infos := get_topic_infos(node, endpoint_type, topic_name, timeout)) is not None:
        qos_profile = fix_qos_profile(topic_infos[0].qos_profile)
        return qos_profile

    return None


def get_topic_infos(node: Node, endpoint_type: TopicEndpointTypeEnum, topic_name: str, timeout: float = -1.0):
    if endpoint_type == TopicEndpointTypeEnum.PUBLISHER:
        get_info_fn = node.get_publishers_info_by_topic
    elif endpoint_type == TopicEndpointTypeEnum.SUBSCRIPTION:
        get_info_fn = node.get_subscriptions_info_by_topic
    else:
        node.get_logger().error("Invalid endpoint type for topic {}".format(topic_name))
        return None

    start_time = perf_counter()
    while rclpy.ok():
        if timeout > 0.0 and perf_counter() - start_time > timeout:
            return None

        topic_infos = get_info_fn(node.resolve_topic_name(topic_name))
        if len(topic_infos) > 0:
            return topic_infos

        sleep(0.1)


def get_msg_type_from_str(msg_type_str: str):
    """
    E.g. "std_msgs/msg/String" -> std_msgs.msg.String
    """

    path = pathlib.Path(msg_type_str)
    module, msg = str(path.parent).replace("/", "."), path.name

    return import_module(module).__dict__[msg]
