from example_interfaces.action import Fibonacci

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import multiprocessing
import time

class ActionServerWrapperProcess(multiprocessing.Process):

    def __init__(self, exit_future):
        multiprocessing.Process.__init__(self)
        self._node = rclpy.create_node('minimal_action_server')
        self._executor = MultiThreadedExecutor()
        self._exit_future = exit_future
        
        self._action_server = ActionServer(
            self._node,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

    def goal_callback(self, goal_request):
        """Accepts or rejects a client request to begin an action."""
        # This server allows multiple goals in parallel
        self._node.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accepts or rejects a client request to cancel an action."""
        self._node.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Executes a goal."""
        self._node.get_logger().info('Executing goal...')

        # Append the seeds for the Fibonacci sequence
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        # Start executing the action
        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._node.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            # Update Fibonacci sequence
            feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            self._node.get_logger().info('Publishing feedback: {0}'.format(feedback_msg.sequence))

            # Publish the feedback
            goal_handle.publish_feedback(feedback_msg)

            # Sleep for demonstration purposes
            #time.sleep(0.5)

        goal_handle.succeed()

        # Populate result message
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence

        self._node.get_logger().info('Returning result: {0}'.format(result.sequence))

        return result


    def run(self):
        print("Server is entering...")

        while not self._exit_future.done():
            rclpy.spin_until_future_complete(
                self._node, 
                self._exit_future,
                executor=self._executor,
                timeout_sec=1)


        self._action_server.destroy()
        self._executor.shutdown()
        self._node.destroy_node()        
       
        print('Server is exiting...')