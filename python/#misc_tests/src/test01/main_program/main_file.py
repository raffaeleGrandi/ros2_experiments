import rclpy
from rclpy.node import Node as Ros2Node
from rclpy.callback_groups import ReentrantCallbackGroup as Reentrant # ReentrantCallbackGroup allows callbacks to be executed in parallel without restriction.
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String as StringMsg

import time
from threading import Thread


class TestNode(Ros2Node):
    
    def __init__(self, node_name, exit_event):
        super().__init__(node_name)
        self._exit_event = exit_event
        self._exec = Thread(target=self._execute)
        self._exec.start()

    def _execute(self):
        self.get_logger().info(f"Running {self.get_name()} for 3 seconds")
        self.get_logger().info(f"Testing exit event done: {self._exit_event.done()}")
        time.sleep(2)
        self._exit_event.set_result(None)
        self.get_logger().info(f"Testing exit event done: {self._exit_event.done()}")
        self.get_logger().info(f"Exiting execute method")


def main():

    exit_event = Future()

    rclpy.init()
    ros2_executor = MultiThreadedExecutor() # it is necessary to execute the ROS callbacks

    node_name = 'TestNode'
    node = TestNode(node_name, exit_event)
    
    rclpy.spin_until_future_complete(node, exit_event, executor=ros2_executor)

    node.destroy_node()
    rclpy.shutdown()

    print("Exiting ROS...")