import argparse
import time
from utils.channelAbs import ChannelAbs
from utils.constants import Constants

import rclpy
from rclpy.node import Node as Ros2Node
from rclpy.callback_groups import ReentrantCallbackGroup as Reentrant # ReentrantCallbackGroup allows callbacks to be executed in parallel without restriction.
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


    def publish_message(self, msg):
        r"""
        The function implements the abstract method of the inherited class ChannelAbs.
        It publishes the message using the publisher of the channel built 
        by the create_channel procedure.
        """

        if not (self.publisher is None):
            self.publisher.publish(msg)


    def channel_callback(self, msg):
        r"""
        This function is the channel's callback. It is selected as callback during 
        the creation of the channel subscriber. It is managed by the ROS2 MultiThreadExecutor.
        When a message is received by the Sender node it is suddenly sent back.
        :param msg: the StringMsg received by the channel's subscriber
        """
        self._logger.info(f"Channel [{self.channel_ID}] received {msg.data}")
        self.publish_message(msg)



class Receiver(Ros2Node):
    r"""
    The Receiver class is created by inheriting from the ROS2 Node class.
    It is responsible of the creation of all the structures used for the 
    communication with the Sender node, including the topics and the Channel's
    entities.
    The received message are sent back to the Sender node without modifications.
    The 'global communications' are managed by using a list of Channels, 
    created and populated by the initialization method.
    """

    def __init__(self, name, num_channels, activation_delay):
        r"""
        The function is devoted to the initialization of the node and the 
        creation of the communication channels.
        The communication channels are stored in a list and created with the
        function 'create_channel'.
        :param name: unique name of the node
        :param num_channels: the number of channels used by the node to send/receive messages
        :param activation_delay: the delay after which the node is active to send messages
        """

        super().__init__(node_name=name)
        self._num_channels = num_channels        
        self._activation_delay = activation_delay
        self._activation_timer = self.create_timer(self._activation_delay, self._activation_callback) # it is used to activate the execution of the node
        self._destroy_delay = Constants.System.DESTROY_DELAY 
        self._qos_depth = Constants.System.SYSTEM_DEFAULT_QOS
        self._channels_list = []
        self._node_creation_time = time.time()
        self._sys_sub = self.create_subscription(
                            StringMsg, Constants.Comms.SYS_TOPIC, 
                            self._sys_callback, qos_profile=self._qos_depth, 
                            callback_group=Reentrant())

        for ch_id in range(num_channels):
            self._create_channel(ch_id)


    def _create_channel(self, ch_id):
        r"""
        The function is used to create a single Channel entity with the publisher,
        the subscriber and the related topics.
        NB: input topic and output topic must be the same of the Sender node but
        in inverted order.
        :param ch_id: channel identifier
        """
        receiver_in_topic =  f"{Constants.Comms.SENDER_OUTPUT_TOPIC_TAG}_ch{ch_id}"
        receiver_out_topic = f"{Constants.Comms.SENDER_INPUT_TOPIC_TAG}_ch{ch_id}"

        channel = Channel(ch_id, receiver_in_topic, receiver_out_topic, self.get_logger())

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


    def explicit_destroy(self):
        time.sleep(self.destroy_delay)
        for i_ch in self.channels_list:
            self.destroy_publisher(i_ch.get_publisher())
            self.destroy_subscription(i_ch.get_subscriber())
        self.destroy_node()

    #----->  CALLBACKS

    '''
    The callback is related to the activation_timer
    '''
    def activation_callback(self):
        # check if executor is assigned
        if self.executor is None:
            self.get_logger().info("Node {} not ready, executor absent. Waiting for {}".format(self.get_name(), self.activation_delay))
        else:
            self.destroy_timer(self.activation_timer)
            self.get_logger().info("Node {} ready, execution starts".format(self.get_name()))



def args_init():
    parser = argparse.ArgumentParser(description="TEST 02 receiver node")
    parser.add_argument("-n", "--name", dest='name', required=True, type=str, help="Name to assign to the node (must be unique in context)")
    parser.add_argument("-c", "--num_channels", dest='ch_num', required=False, default=1, type=int, help="Number of pub/sub couples to create")
    parser.add_argument("-a", "--activation-delay", dest='activation_delay', required=False, default=1, type=int, help="Time after which the node starts its execution")
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
    executor = MultiThreadedExecutor() # it is necessary to execute the ROS callbacks

    node = Receiver(parsed_args.name,
                    parsed_args.ch_size,
                    parsed_args.activation_delay)

    rclpy.spin(node, executor=executor)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
