#!/usr/bin/env python
__author__ = 'patrick'
import threading


class Debugger:

    def __init__(self, robot):
        self.robot = robot
        self.user_input = [""]

    def getInput(self):
        while self.user_input[0] != "q":
            self.user_input[0] = raw_input()
            if self.user_input[0] == "ps":
                for action in self.robot._actionsStack_:
                    print(str(action[0]) +" " + str(action[1]))
            elif self.user_input[0] == "stop":
                print(self.robot._stopCurrentAction_)
            elif self.user_input[0] == "unload":
                self.robot.current_load = 0
            elif self.user_input[0] == "pickers":
                for i in self.robot.picker_robots:
                    data = i.split(",")
                    print(data[0])
            elif self.user_input[0] == "load":
                print(self.robot.current_load)
            elif self.user_input[0] == "state":
                print(self.robot.state)
        self._thread.exit()

    def start(self):
        inputThread = threading.Thread(target=self.getInput, args=())
        inputThread.daemon = True
        inputThread.start()