#!/usr/bin/env python

import threading
from collections import deque
import rospy
from std_msgs.msg import*
import os


class Carrier_Queue:

    def __init__(self):
        rospy.init_node("Carrier_Queue")

        # picker_queue is a FIFO queue contains the  ids of full pickers
        self.picker_queue = deque([])

        # targeted_pickers is a list of pickers that carriers are currently moving towards for collection
        self.targeted_pickers = []

        self.lock = threading.RLock()
        self.picker_robots = ["0,0,0","0,0,0","0,0,0","0,0,0","0,0,0","0,0,0"]
        self.max_load = 20
        self.total_kiwis_collected = 0
        self.total_collections = 0

        self.queue_pub = rospy.Publisher("carrier_allocation_response",String, queue_size=10)
        self.queue_sub = rospy.Subscriber("carrier_allocation_request", String, self.request_callback)
        self.picker_sub = rospy.Subscriber("picker_position", String, self.picker_callback)

    """
    @function
    @parameter: message

    Callback when a carrier makes a request
    If carrier is waiting, and there is something in the queue, it will post the id of the next waiting picker to the carrier
    If it arrived at the picker, it will remove the picker id from the targeted queue
    """
    def request_callback(self, message):
        carrier_id = message.data.split(",")[0]
        requested_action = message.data.split(",")[1]
        next_robot_id = message.data.split(",")[2]

        # if the carrier is waiting
        if(requested_action == "waiting"):
            if len(self.picker_queue) > 0:
                self.lock.acquire()
                try:
                    self.queue_pub.publish(str(carrier_id) + "," + str(self.get_next_in_queue()))
                    self.post_to_file()
                finally:
                    self.lock.release()

        # if the carrier has arrived at the picker
        elif (requested_action == "arrived"):
            if(next_robot_id != "None"):
                self.lock.acquire()
                try:
                    self.targeted_pickers.remove(int(next_robot_id))
                    self.total_kiwis_collected += self.max_load
                    self.total_collections += 1
                    self.post_to_file()
                finally:
                    self.lock.release()

    """
    @function

    Adds the picker id to the targeted queue
    Removes the next picker id in the queue
    Returns the picker id of the next robot on the queue
    """
    def get_next_in_queue(self):
        self.lock.acquire()
        try:
            for pickerid in self.picker_queue:
                if(pickerid not in self.targeted_pickers):
                    self.targeted_pickers.append(pickerid)
                    self.picker_queue.popleft()
                    return pickerid
        finally:
            self.lock.release()

    """
    @function
    @parameter: message

    Sets the position of the picker robot
    If it is full and not in any of the queues, it will append it to the picker queue
    """
    def picker_callback(self, message):
        picker_index = int(message.data.split(',')[0])
        self.picker_robots[picker_index] = message.data.split(',')[1] + "," + message.data.split(',')[2] + "," + message.data.split(',')[4]  # Should add element 3 here which is theta

        if int(self.picker_robots[picker_index].split(',')[2]) == self.max_load:
            self.lock.acquire()
            try:
                if picker_index not in self.picker_queue:
                    if picker_index not in self.targeted_pickers:
                        self.picker_queue.append(picker_index)
            finally:
                self.lock.release()


    """
    @function

    Posts the carriers status to a file for the gui
    """
    def post_to_file(self):
        fn = os.path.join(os.path.dirname(__file__), "carrier.que")
        output_file = open(fn, "w")
        output_file.write("Carrier Queue\n")
        output_file.write(str(list(self.picker_queue)) + "\n")
        output_file.write(str(self.targeted_pickers) + "\n")
        output_file.write(str(self.total_kiwis_collected) + "\n")
        output_file.write(str(self.total_collections) + "\n")
        output_file.close()