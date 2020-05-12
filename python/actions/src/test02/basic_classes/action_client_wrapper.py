from example_interfaces.action import Fibonacci
from action_msgs.msg import GoalStatus

from rclpy.action import ActionClient

import threading
import time


class ActionClientWrapper(threading.Thread):        

    def __init__(self, node, exit_future):
        threading.Thread.__init__(self)
        self._node = node
        self._exit_future = exit_future

        self._action_client = ActionClient(node, Fibonacci, 'fibonacci')      

    def send_goal(self):
        self._node.get_logger().info('Waiting for action server...')

        self._action_client.wait_for_server()

        goal_msg = Fibonacci.Goal()
        goal_msg.order = 10

        self._node.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._node.get_logger().info('Goal rejected :(')
            return

        self._node.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        self._node.get_logger().info('Received feedback: {0}'.format(feedback.feedback.sequence))

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._node.get_logger().info('Goal succeeded! Result: {0}'.format(result.sequence))
        else:
            self._node.get_logger().info('Goal failed with status: {0}'.format(status))

        # Shutdown after receiving a result
        # rclpy.shutdown()

        self._exit_future.set_result(None)    
    
    def destroy(self):
        self._action_client.destroy()
        
    def run(self):
        print("Client is entering...")
        self.send_goal()

        while not self._exit_future.done():
            time.sleep(1)
            print("Client is running...")
        
        print('Client is exiting...')