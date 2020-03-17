import rclpy
from rclpy.node import Node as Ros2Node
from rclpy.callback_groups import ReentrantCallbackGroup as Reentrant # ReentrantCallbackGroup allows callbacks to be executed in parallel without restriction.
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String as StringMsg

import argparse


class Receiver(Ros2Node):

    def __init__(self, name, topic):
        super().__init__(node_name=name)
        self._topic = topic
        self._message_type = StringMsg
        self.create_subscription(self._message_type, self._topic, 
            self._callback, callback_group=Reentrant(), qos_profile=0)
        self.get_logger().info(f"Receiver {self.get_name()} initialized on topic {self._topic}")


    def _callback(self, received_msg):
        self.get_logger().info(f"{self.get_name()} received message: {received_msg.data}")


def parse_init_args():
    parser = argparse.ArgumentParser(description="Basic_test_01 receiver parameters")
    parser.add_argument("-n", "--name", dest='name', required=True, type=str, help="Node name (unique)")    
    parser.add_argument("-t", "--topic", dest='topic', required=True, type=str, help="Topic")
    return parser.parse_args()


def main():
    
    parsed_args = parse_init_args()

    rclpy.init()
    executor = MultiThreadedExecutor() # it is necessary to execute the ROS callbacks
    
    receiver_node = Receiver(parsed_args.name, parsed_args.topic)

    rclpy.spin(receiver_node, executor=executor)

    receiver_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
