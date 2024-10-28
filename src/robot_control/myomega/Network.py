from abc import ABCMeta, abstractmethod


class Network(metaclass=ABCMeta):

    @abstractmethod
    def create_net(self):
        """
        创建网络。
        Create the network.
        """
