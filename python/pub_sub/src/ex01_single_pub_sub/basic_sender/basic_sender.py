import rclpy
from rclpy.node import Node as Ros2Node
from rclpy.callback_groups import ReentrantCallbackGroup as Reentrant # ReentrantCallbackGroup allows callbacks to be executed in parallel without restriction.
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String as StringMsg

import random
import argparse
import time

STOP_MESSAGE = '!STOP!'

class Sender(Ros2Node):

    def __init__(self, name, topic, msg_size, pub_time, max_iters, exit_event):
        super().__init__(node_name=name)
        self._topic = topic
        self._message_type = StringMsg
        self._msg_size = msg_size
        self._pub_timer = self.create_timer(pub_time, self._send_callback)
        self._publisher = self.create_publisher(self._message_type, self._topic, qos_profile=0)
        self._max_iters = max_iters if max_iters > 0 else float('inf')
        self._curr_iters = 0
        self._exit_event = exit_event
        

    def _send_callback(self):
        if self._curr_iters < self._max_iters:
            msg = ''.join([chr(random.randint(65,90)) for _ in range(self._msg_size)])
            self.get_logger().info(f"publishing message [{self._curr_iters+1}/{self._max_iters}]: {msg}")
            ros2msg = StringMsg(data=msg)
            self._publisher.publish(ros2msg)
            self._curr_iters += 1
        else:
            self.get_logger().info("Exiting... max num iters reached")
            self._publisher.publish(StringMsg(data=STOP_MESSAGE))
            time.sleep(0.5)
            self.destroy_timer(self._pub_timer)
            self.destroy_publisher(self._publisher)
            self._exit_event.set_result(None)
            
            

def parse_init_args():
    parser = argparse.ArgumentParser(description="Basic_test_01 sender parameters")
    parser.add_argument("-n", "--name", dest='name', required=True, type=str, help="Node name (unique)")    
    parser.add_argument("-t", "--topic", dest='topic', required=True, type=str, help="Topic")
    parser.add_argument("-s", "--msg-size", dest='msg_size', required=False, default=2, type=int, help="Message size (#chars)")    
    parser.add_argument("-p", "--pub-time", dest='pub_time', required=False, default=0.2, type=float, help="Publishing time (seconds)")
    parser.add_argument("-i", "--max-iters", dest='num_iters', required=False, default=1, type=int, help="Iterations number (0 = infinite)")
    return parser.parse_args()


def main():
    
    parsed_args = parse_init_args()
    
    rclpy.init()    
    executor = SingleThreadedExecutor() # executes the ROS callbacks

    exit_event = rclpy.Future()

    sender_node = Sender(parsed_args.name,
                         parsed_args.topic,
                         parsed_args.msg_size,
                         parsed_args.pub_time,                         
                         parsed_args.num_iters,
                         exit_event)

    rclpy.spin_until_future_complete(sender_node, exit_event, executor=executor)

    sender_node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()