import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import threading
import time

from basic_classes.action_client_wrapper import ActionClientWrapper
from basic_classes.action_server_wrapper import ActionServerWrapper
   

def main():

    rclpy.init(args=None)

    node = rclpy.create_node('minimal_action_node')
    executor = MultiThreadedExecutor()
    exit_future = rclpy.Future()
    

    asw = ActionServerWrapper(node, exit_future)
    asw.start()

    time.sleep(0.5)  # without this instruction this error is thrown at the end of the program: 
    # PyCapsule_GetPointer called with invalid PyCapsule object #3

    acw = ActionClientWrapper(node, exit_future)
    acw.start()

    while not exit_future.done():
        rclpy.spin_until_future_complete(
            node, 
            exit_future,
            executor=executor,
            timeout_sec=1)

    acw.destroy()
    asw.destroy()

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()