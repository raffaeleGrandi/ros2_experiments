import rclpy
from rclpy.node import Node as Ros2Node
from rclpy.callback_groups import ReentrantCallbackGroup as Reentrant # ReentrantCallbackGroup allows callbacks to be executed in parallel without restriction.
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String as StringMsg


from concurrent.futures import ThreadPoolExecutor


class TestNode(Ros2Node):
    
    def __init__(self, node_name):
        super().__init__(node_name)

        self.get_logger().info(f"Running {self.get_name()}")


def main():
    rclpy.init()
    ros2_executor = MultiThreadedExecutor() # it is necessary to execute the ROS callbacks

    node_name = 'TestNode'
    node = TestNode(node_name)

    try:
        rclpy.spin(node, executor=ros2_executor)
    except Exception as e:
        print("Exiting ROS")
