#!/usr/bin/env python

import threading
from collections import deque
import rospy
from std_msgs.msg import*


class My_Singleton:

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
        rospy.init_node("Singleton")
        self.picker_queue = deque([])
        self.targeted_pickers = []
        self.lock = threading.RLock()
        self.picker_robots = ["0,0,0","0,0,0","0,0,0","0,0,0","0,0,0","0,0,0"]
        self.max_load = 20


        self.queue_pub = rospy.Publisher("carrier_allocation_response",String, queue_size=10)
        self.queue_sub = rospy.Subscriber("carrier_allocation_request", String, self.request_callback)

        self.picker_sub = rospy.Subscriber("pickerPosition", String, self.pickerCallback)

        rospy.init_node('Singleton')



    """
    @function
    @parameter: message

    Sets the position of carrier robots received from messages on the topic
    """
    def request_callback(self, message):
        carrier_id = message.data.split(",")[0]
        requested_action = message.data.split(",")[1]
        next_robot_id = message.data.split(",")[2]
        if(requested_action == "waiting"):
            if len(self.picker_queue) > 0:
                self.printLists()
                self.queue_pub.publish(str(carrier_id) + "," + str(self.get_next_in_queue()))
        elif (requested_action == "arrived"):
            if(next_robot_id != "None"):
                self.targeted_pickers.remove(int(next_robot_id))
        # carrier receives (own_id, picker_id)


    """
    @function

    Sets the next robot in the picker queue
    """
    def get_next_in_queue(self):
        self.lock.acquire()
        try:
            print(str(self) + " next")
            for pickerid in self.picker_queue:
                if(pickerid not in self.targeted_pickers):
                    self.printLists()
                    # self.next_robot_id = pickerid
                    self.targeted_pickers.append(pickerid)
                    self.picker_queue.popleft()
                    self.printLists()
                    return pickerid
        finally:
            self.lock.release()

    """
    @function
    @parameter: message

    Sets the position of picker robots received from messages on the topic
    """
    def pickerCallback(self, message):
        picker_index = int(message.data.split(',')[0])
        self.picker_robots[picker_index] = message.data.split(',')[1] + "," + message.data.split(',')[2] + "," + message.data.split(',')[4]  # Should add element 3 here which is theta

        if int(self.picker_robots[picker_index].split(',')[2]) == self.max_load:
            self.lock.acquire()
            try:
                if picker_index not in self.picker_queue:
                    if picker_index not in self.targeted_pickers:
                        print("Added picker " + str(picker_index) + " to queue by queue master")
                        self.picker_queue.append(picker_index)
                        self.printLists()
            finally:
                self.lock.release()

    def printLists(self):
        print("picker queue is " + str(self.picker_queue))
        print("targeted queue is " + str(self.targeted_pickers))
        pass