import argparse
import time
import threading
import numpy as np
from concurrent.futures import ThreadPoolExecutor

from utils.channelAbs import ChannelAbs
from utils.constants import Constants

import rclpy
from rclpy.task import Future
from rclpy.node import Node as Ros2Node
from rclpy.callback_groups import ReentrantCallbackGroup as Reentrant
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String as StringMsg


class Channel(ChannelAbs):
    r"""
    The class Channel defines a bidirectional communication channel, built as 
    a sort of container for a publisher, a subscriber and their related topics
    for I/O communications
    """

    def __init__(self, ch_id, in_topic_name, out_topic_name, node_logger):
        super().__init__(ch_id, in_topic_name, out_topic_name)
        self._logger = node_logger

    # When the Sender executes the publisher distruction, self.publisher 
    # becomes None, the method exit without publish and the ThreadPoolExecutor 
    # no more executes the function.
    def publish_message(self, msg):
        r"""
        The function implements the abstract method of the inherited class ChannelAbs.
        It publishes the message using the publisher of the channel built 
        by the create_channel procedure.
        During the publishing phase the current system time is collected in 
        order to measure the time elapsed between sending and receiving the message
        :param msg: the StringMsg to be published by the channel publisher
        """

        self.set_last_pub_time(time.time())
        try:
            self.publisher.publish(msg)
        except Exception as e:
            self._logger.error(f"Ch{self.channel_ID} unable to publish")


    def channel_callback(self, msg):
        r"""
        This function is the channel's callback. It is selected as callback during 
        the creation of the channel subscriber. 
        It is managed by the ROS2 MultiThreadExecutor
        :param msg: the StringMsg received by the channel's subscriber
        """

        self._logger.info(f"Ch{self.channel_ID} received {msg.data}")
        i_stats = time.time() - self.get_last_pub_time()
        # self._logger.info(f"Stats for msg {msg.data} = {i_stats:.3f}")
        self.set_stats(i_stats)



class Sender(Ros2Node):
    r"""
    The Sender class is created by inheriting from the ROS2 Node class.
    It is responsible of the creation of all the structures used for the 
    communication with the Receiver node, including the topics, the Channel entities
    and the creation of all messages sent to the Receiver.
    The messages are randomly created with the create_ros2msg function and are 
    sent to the Receiver node. The 'global communications' are managed by using 
    a list of Channels, created and populated by the initialization method.
    """

    def __init__(self, name, num_channels, msg_size, max_num_iter,
                    activation_delay, publishing_delay, out_file=''):
        r"""
        The function is devoted to the initialization of the node and the 
        creation of the communication channels.
        The communication channels are stored in a list and created with the
        function 'create_channel'.
        :param name: unique name of the node
        :param num_channels: the number of channels used by the node to send/receive messages
        :param msg_size: the message size is the length of the message to send 
            in terms of number of characters.
        :param max_num_iter: the max number of iterations represents how many 
            messages are sent by each channel in sequence
        :param publishing_delay: the delay after which a channel publishes a message
        :param activation_delay: the delay after which the node is active to send messages
        :param out_file: specifies the file used to collect statistics
        """

        super().__init__(node_name=name)        
        self._num_channels = num_channels
        self._msg_size = msg_size if (0 < msg_size < Constants.Comms.MAX_MESSAGE_SIZE) else Constants.Comms.MAX_MESSAGE_SIZE
        self._max_num_iter = max_num_iter        
        self._activation_delay = activation_delay
        self._activation_timer = self.create_timer(self._activation_delay, self._activation_callback) # it is used to activate the execution of the node
        self._pub_delay = publishing_delay
        self._destroy_delay = Constants.System.DESTROY_DELAY 
        self._qos_depth = Constants.System.SYSTEM_DEFAULT_QOS
        self._out_file = out_file
        self._channels_list = []
        self._node_creation_time = time.time()
        self._sys_pub = self.create_publisher(StringMsg, Constants.Comms.SYS_TOPIC, qos_profile=self._qos_depth)

        for ch_id in range(num_channels):
            self._create_channel(f"{ch_id:0>3}")


    def _create_channel(self, ch_id):
        r"""
        The function is used to create a single Channel entity with a publisher,
        a subscriber and the their related topics.
        :param ch_id: channel identifier
        """
        sender_in_topic = f"{Constants.Comms.SENDER_INPUT_TOPIC_TAG}_ch{ch_id}"
        sender_out_topic = f"{Constants.Comms.SENDER_OUTPUT_TOPIC_TAG}_ch{ch_id}"

        channel = Channel(ch_id, sender_in_topic, sender_out_topic, self.get_logger())

        channel.publisher = self.create_publisher(
                                StringMsg, 
                                channel.output_topic, 
                                qos_profile=self._qos_depth)

        channel.subscriber = self.create_subscription(
                                StringMsg, 
                                channel.input_topic, 
                                channel.channel_callback, 
                                qos_profile=self._qos_depth, 
                                callback_group=Reentrant())

        self._channels_list.append(channel)


    def _create_ros2msg(self):
        r"""
        Creates the ROS2 message as a lowercase alphabetical random string 
        with variable length. The length is selected by the user at launch time
        with '-s' parameter.
        """

        payload = ''.join([chr(np.random.randint(65,90)) for _ in range(self._msg_size)])        
        return StringMsg(data=payload)


    def _execute(self):
        r"""
        The method is the core function for the node execution.
        It is launched by the activation_callback and starts to send messages.
        A ThreadPoolExecutor is created to manage the channel publishing function.
        For each iteration a new random message is generated and a ThreadPoolExecutor 
        executes the publish_message method of each channel in order to send 
        the message.
        """

        max_workers = len(self._channels_list)
        executor = ThreadPoolExecutor(max_workers=max_workers)

        for i_iter in range(self._max_num_iter):
            print(f"\nRunning iteration # {i_iter}")
            for i_channel in self._channels_list:
                time.sleep(self._pub_delay)
                i_ros2msg = self._create_ros2msg()
                self.get_logger().info(f"Ch{i_channel.channel_ID} sends: {i_ros2msg.data}")                
                executor.submit(i_channel.publish_message, i_ros2msg)
        
        print("End execution")
        print("Saving statistics")
        try:            
            self._save_statistics()
        except Exception as e:
            self.get_logger().error(f"Unable to save statistics: {e}")

    def _activation_callback(self):
        r"""
        In order to wait the correct deployment of all the ROS resources invoked, 
        a proper 'activation_timer' is created during the init phase of the Sender node.
        This method is the callback connected with the activation_timer and manages
        the 'living phase' of the node
        """
        # check if ROS2 executor is assigned
        if self.executor is None:
            self.get_logger().info(f"Node {self.get_name()} not ready, executor absent. " \
                "Waiting for {self._activation_delay}.")
        else:            
            self.get_logger().info(f"Node {self.get_name()} ready, execution starts...")
            self.destroy_timer(self._activation_timer)            
            self._execute()
            print("Execution ENDs, sending STOP message")
            self._sys_pub.publish(StringMsg(data=Constants.Comms.STOP_MESSAGE))
            print("Activation callback ENDs")
            self._explicit_destroy()
            

    def _save_statistics(self):
        r"""
        Helper method to save statistics regarding the time elapsed between the 
        sending and the receiving of a message between Sender and Receiver nodes
        """
        data = np.vstack([np.array(ch.get_stats()).transpose() for ch in self._channels_list]) # get data from each channel
        if data != []:
            self.get_logger().info("Saving results in {}...".format(self._out_file))
            np.savetxt(self.out_file, data.transpose(), delimiter="\t")
            self.get_logger().info("Results saved!")
        else:
            self.get_logger().info("No stats available!")


    def _explicit_destroy(self):
            r"""
            The function is used to explicitely destroy all the structures of the node,
            since sometimes the automatic procedure does not work and some structures remain pending.
            The function waits for a destroy delay before executing.
            """
            time.sleep(self._destroy_delay)
            print('Destroying channels')
            for i_ch in self._channels_list:
                try:
                    self.destroy_publisher(i_ch.publisher)            
                    self.destroy_subscription(i_ch.subscriber)
                except Exception as e:
                    print(f"Unable to destroy channel {i_ch.channed_ID}")
            print("Destroying system publisher")
            self.destroy_publisher(self._sys_pub)
            print('Destroying node')
            try:
                self.destroy_node()
            except Exception as e:
                print("Unable to destroy node")                
            rclpy.shutdown()
            

def args_init():
    r"""
    Helper method to parse the input arguments
    """
    parser = argparse.ArgumentParser(description="Sender node of multi a publisher/subscriber structure")
    parser.add_argument("-n", "--name", dest='name', required=True, type=str, help="Name to assign to the node (must be unique in context)")
    parser.add_argument("-c", "--num-channels", dest='ch_num', required=False, default=1, type=int, help="Number of pub/sub couples to create")
    parser.add_argument("-s", "--message-size", dest='msg_size', required=False, default=4, type=int, help="Size of the message to publish in bytes")
    parser.add_argument("-i", "--iterations", dest='iterations', required=True, type=int, help="Number of iterations to perform")
    parser.add_argument("-a", "--activation-delay", dest='activation_delay', required=False, default=1, type=int, help="Time after which the node starts its execution")
    parser.add_argument("-p", "--publishing-delay", dest='publishing_delay', required=False, default=0, type=int, help="Time after which the node publish another message")
    parser.add_argument("-o", "--output", dest='output', required=False, default='', type=str, help="File used to collect statistics")
    return parser.parse_args()


def main(args = None):
    r"""
    The main function is defined as the default entry point of the receiver 
    in the package. ROS2 is initialized, the proper callbacks manager is 
    created (MultiThreadExecutor). The node is created, configured
    with the parameters given by the user at command line and finally started 
    (spinned)
    """
    parsed_args = args_init()
    rclpy.init(args = args)
    ros2_executor = MultiThreadedExecutor() # it is necessary to execute the ROS callbacks

    node = Sender(parsed_args.name,
                  parsed_args.ch_num,
                  parsed_args.msg_size,
                  parsed_args.iterations,
                  parsed_args.activation_delay,
                  parsed_args.publishing_delay,
                  parsed_args.output)

    try:
        rclpy.spin(node, executor=ros2_executor)
    except Exception as e:
        print("Exiting ROS")


if __name__ == "__main__":
    main()
