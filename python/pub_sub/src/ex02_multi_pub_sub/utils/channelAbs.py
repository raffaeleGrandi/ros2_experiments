from abc import ABC, abstractmethod

class ChannelAbs(ABC):

    def __init__(self, ch_id, in_topic_name, out_topic_name):
        self._channel_ID = ch_id
        self._publisher = None
        self._subscriber = None
        self._in_topic = in_topic_name
        self._out_topic = out_topic_name
        self._pub_tic = 0.0
        self._stats = []

    @property
    def channel_ID(self):
        return self._channel_ID

    @property
    def publisher(self):
        return self._publisher
    
    @publisher.setter
    def publisher(self, pub):
        self._publisher = pub    

    @property
    def subscriber(self):
        return self._subscriber

    @subscriber.setter
    def subscriber(self, sub):
        self._subscriber = sub

    @property
    def input_topic(self):
        return self._in_topic

    @property
    def output_topic(self):
        return self._out_topic

    def get_last_pub_time(self):
        return self._pub_tic

    
    def set_last_pub_time(self, pub_time):
        self._pub_tic = pub_time

    def get_stats(self):
        return self._stats

    def set_stats(self, new_stats):
        self._stats.append(new_stats)

    @abstractmethod
    def publish_message(self, msg):
        pass

    @abstractmethod
    def channel_callback(self, msg):
        pass
