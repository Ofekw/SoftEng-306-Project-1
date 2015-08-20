import threading
from collections import deque
import rospy
from std_msgs.msg import*


class My_Singleton():

    # def __init__(self, decorated):
    #     self._decorated = decorated
    #     self.picker_queue = deque([])
    #     self.targeted_pickers = []
    #     self.lock = threading.RLock()
    #
    # def Instance(self):
    #     try:
    #         return self._instance
    #     except AttributeError:
    #         self._instance = self._decorated()
    #         return self._instance

    def __init__(self):
        self.picker_queue = deque([])
        self.targeted_pickers = []
        self.lock = threading.RLock()

        self.queue_pub = rospy.Publisher("carrier_allocation",String, queue_size=10)
        self.queue_sub = rospy.Subscriber("carrier_allocation", String, self.carrier_callback)


    """
    @function
    @parameter: message

    Sets the position of carrier robots received from messages on the topic
    """
    def carrierCallback(self, message):
        self.carrier_robots[int(message.data.split(',')[0])] = message.data.split(',')[1] + "," + message.data.split(',')[2] #+ "," + message.data.split(',')[4]  # Should add element 4 here which is theta
        # print("Carrier array")
        # print ', '.join(self.carrier_robots)



    # __instance = None
    #
    # def __new__(cls):
    #     if cls.__instance == None:
    #         __instance = type.__new__(cls)
    #         __instance.name = "The one"
    #     return __instance
    #
    # def __call__(self):
    #     raise TypeError('Singletons must be accessed through `Instance()`.')
    #
    # def __instancecheck__(self, inst):
    #     return isinstance(inst, self._decorated)

    def get_lock(self):
        return self.lock

    def get_picker_queue(self):
        return self.picker_queue

    def get_targeted_pickers(self):
        return self.targeted_pickers
    