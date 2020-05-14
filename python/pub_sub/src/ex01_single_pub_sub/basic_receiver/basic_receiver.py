import rclpy
from rclpy.node import Node as Ros2Node
from rclpy.callback_groups import ReentrantCallbackGroup as Reentrant # ReentrantCallbackGroup allows callbacks to be executed in parallel without restriction.
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from std_msgs.msg import String as StringMsg

import argparse

STOP_MESSAGE = '!STOP!'

class Receiver(Ros2Node):

    def __init__(self, name, topic, exit_event):
        super().__init__(node_name=name)
        self._topic = topic
        self._message_type = StringMsg
        self._sub = self.create_subscription(self._message_type, self._topic, 
            self._callback, callback_group=Reentrant(), qos_profile=0)        
        self._exit_event = exit_event
        self._msgs_counter = 0

        self.get_logger().info(f"Receiver initialized on topic {self._topic}")


    def _callback(self, received_msg):
        received_data = received_msg.data        
        
        if received_data != STOP_MESSAGE:
            self._msgs_counter += 1
            self.get_logger().info(f"received message #{self._msgs_counter}: {received_data}")
        else:
            self.get_logger().info(f"received STOP message: {received_data}")
            self.get_logger().info("Exiting...")
            self.destroy_subscription(self._sub)
            self._exit_event.set_result(None)


def parse_init_args():
    parser = argparse.ArgumentParser(description="Basic_test_01 receiver parameters")
    parser.add_argument("-n", "--name", dest='name', required=True, type=str, help="Node name (unique)")    
    parser.add_argument("-t", "--topic", dest='topic', required=True, type=str, help="Topic")
    return parser.parse_args()


def main():
    
    parsed_args = parse_init_args()

    rclpy.init()
    executor = SingleThreadedExecutor() # executes the ROS callbacks
    # executor = MultiThreadedExecutor() # executes the ROS callbacks

    exit_event = rclpy.Future()
    
    receiver_node = Receiver(parsed_args.name, parsed_args.topic, exit_event)

    rclpy.spin_until_future_complete(receiver_node, 
                                     exit_event, 
                                     executor=executor)

    receiver_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
