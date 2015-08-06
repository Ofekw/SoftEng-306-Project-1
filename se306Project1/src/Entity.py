#!/usr/bin/env python

import rospy
from std_msgs.msg import*
from geometry_msgs.msg import*
from nav_msgs.msg import*
from sensor_msgs.msg import*
from tf.transformations import *
import math
import ActionInterruptException
import numpy.testing


"""
@class

The Entity class used to represent an Entity in the world stage.
This class should never be instantiated and thus be treated as a
Abstract class.

Robots, Humans, Tractors, Dogs etc should inherit from this.

Can go forward and turn left or right or by a certain angle.
"""
class Entity:

    def __init__(self,r_id,x_off,y_off):

        #declaring the instance variables
        self.robot_id = 0
        self.linearX = 0
        self.angularZ = 0
        self.theta = 0
        self.px = x_off
        self.py = y_off
        self.robot_id = r_id
        self.robot_node_name = ("RobotNode" +str(r_id))
        self.robot_node_identifier = ("robot_"+ str(r_id))

        #array of methods of robot actions
        self._actions_ = {
            0: self.move_forward,
            1: self.goto,
            2: self.turn,
            3: self.stop,
        }

        self._actionsStack_ = []

        #stop current action used inside methods to check if there has been a call to run a new method
        self._stopCurrentAction_ = False

        #variable to track if action is running or not
        self._actionRunning_ = False

        #Node Initiation
        rospy.init_node(self.robot_node_name)

        #setting up publishers and subscribers
        self.RobotNode_stage_pub = rospy.Publisher(self.robot_node_identifier+"/cmd_vel", geometry_msgs.msg.Twist, queue_size=10)
        self.StageOdo_sub = rospy.Subscriber(self.robot_node_identifier+"/odom", nav_msgs.msg.Odometry, self.StageOdom_callback)

        self.RobotNode_cmdvel = geometry_msgs.msg.Twist()
        self.RobotNode_odom = geometry_msgs.msg.Pose2D()

        self.StageLaser_sub = rospy.Subscriber(self.robot_node_identifier+"/base_scan",sensor_msgs.msg.LaserScan,self.StageLaser_callback)
        self.StageLaser_sub = rospy.Subscriber

    """
    @function
    @parameter: Msg msg

    Callback function to update position and other odometry values
    """
    def StageOdom_callback(self,msg):

        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y

        (roll, pitch, yaw) = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))

        self.theta = yaw

        #rospy.loginfo("Current x position: %f" , self.px)
        #rospy.loginfo("Current y position: %f", self.py)
        #rospy.loginfo("Current theta: %f", self.theta)


    def StageLaser_callback(self, msg):
        barCount = 0
        found = False

        #for i in range(0,180):

        for i in range(70, 110):
            if msg.ranges[i]< 4.0:
                action = self._actions_[2], ["right"]
                #check if action already exists in stack, otherwise laser will spam rotates
                if action != self._actionsStack_[-1]:
                    print("adding right to stack")
                    #stop moving foward and add turn action
                    self._stopCurrentAction_ = True
                    self._actionsStack_.append(action)
        #check that all lasers in 0-20 range are not hitting object
        rangeHitting = False
        for i in range(160,180):
            if msg.ranges[i] < 5.0:
                rangeHitting = True

        if not rangeHitting:
            print("No Wall Left")
            action = self._actions_[2], ["left"]
            #check if action already exists in stack, otherwise laser will spam rotates
            if action != self._actionsStack_[-1]:
                #stop moving foward and add turn action
                self._stopCurrentAction_ = True
                self._actionsStack_.append(action)
        else:
            print("Wall Left")

    """
    @function
    @parameter: int dist


    Moves the Entity forward by a certain specified distance
    """
    def move_forward(self, dist):
        """
        Changes the forward velocity of the Entity to 1. It will then move forward, until the distance it has moved forward
        has reached the distance passed to this function.
        """

        #Initiate the distance gained as 0
        dist_gained = 0

        #Iniate a reference to the initial x and y positions
        previousX = self.px
        previousY = self.py


        #While the distance that the Entity has gained has not exceeded the given distance, continue to move the Entity forward
        while (dist_gained < dist and not (self._stopCurrentAction_)):

            #Calculate remaining distance to travel
            distToGo = dist - dist_gained

            #If the remaining distance is less than 1m, then decelerate the Entity. Having a slower moving Entity will
            #provide increased accuracy when stopping
            if (distToGo < 1):
                #Set forward velocity to 0.7m/s
                self.RobotNode_cmdvel.linear.x = 0.7
            else:
                #Set forward velocity to 2.0m/s
                self.RobotNode_cmdvel.linear.x = 2


            #Publish the velocity change
            self.RobotNode_stage_pub.publish(self.RobotNode_cmdvel)

            #Sleep rospy for 2ms
            rospy.sleep(0.02)

            #Calculate the change in x and y positions from the initial x and y positions, prior to the Entity moving
            xDiff = abs(previousX - self.px)
            yDiff = abs(previousY - self.py)

            #Find the distance gained by calculating sqrt(xDiff^2 + yDiff^2)
            dist_gained = math.sqrt(xDiff * xDiff + yDiff * yDiff)

            print("Moving Forward: " + str(distToGo) + "m to go")
            print("Current x pos = " + str(self.px) +"," +str(self.py))


        if self._stopCurrentAction_ == True:
                #stop movement and throw exception
                self.RobotNode_cmdvel.linear.x = 0
                self.RobotNode_stage_pub.publish(self.RobotNode_cmdvel)
                self._stopCurrentAction_ = False
                raise ActionInterruptException.ActionInterruptException("Move Interrupted")
        else:
                #Stop robot by setting forward velocity to 0 and then publish change
                self.RobotNode_cmdvel.linear.x = 0
                self.RobotNode_stage_pub.publish(self.RobotNode_cmdvel)
                #return 0 for succesful finish
                #set action running tracker to false as method finished
                return 0


    """
    @function

    @parameter:String direction

    Turn function which allows the Entity to turn 90 degrees ( a right angle) either left or right.
    """
    def turn(self, direction):

        pi = math.pi


        if (direction == "left"):
            thetaTarg = self.theta + pi/2
            dir = 1
            if (thetaTarg > pi):
                thetaTarg = - pi + (thetaTarg - pi)
        elif (direction == "right"):
            thetaTarg = self.theta - pi/2
            dir = -1
            if (thetaTarg < -pi):
                thetaTarg = pi + (thetaTarg + pi)

        while (abs(self.theta - thetaTarg) > 0.01 and not (self._stopCurrentAction_)):
            thetaDiff = abs(self.theta - thetaTarg)

            #Set the angular velocity to optimal values that don't overshoot pi/2
            if (thetaDiff > 0.5):
                self.RobotNode_cmdvel.angular.z = 2.5 * dir
            elif (thetaDiff > 0.1):
                self.RobotNode_cmdvel.angular.z = 0.3 * dir
            else:
                self.RobotNode_cmdvel.angular.z = 0.02 * dir

            self.RobotNode_stage_pub.publish(self.RobotNode_cmdvel)

            rospy.sleep(0.0001)

            #print("Turning " + direction + " current theta is " + str(self.theta) +", target theta is " + str(thetaTarg))

        if self._stopCurrentAction_ == True:
                self._stopCurrentAction_ = False
                raise ActionInterruptException.ActionInterruptException("Wall hit")
        else:
                #Stop robot by setting forward velocity to 0 and then publish change
                self.RobotNode_cmdvel.angular.z = 0
                self.RobotNode_stage_pub.publish(self.RobotNode_cmdvel)
                #return 0 for succesful finish
                return 0




    """
    @function

    @parameter:float angle

    Turn function which allows the Entity to turn a specified number of degrees either left or right.
    A negative angle would denote a right rotation and vice-versa
    """
    def rotate_relative(self, angle, angle_type):

        if (angle<0):
            dir = -1
        else:
            dir = 1
        pi=math.pi
        #convert degrees to radians
        if (angle_type=="degrees"):
            angle_in_radians = (math.pi/180) *angle
        else:
            angle_in_radians=angle


        thetaTarg = self.theta + angle_in_radians

        if (thetaTarg > pi):
            thetaTarg = - pi + (thetaTarg - pi)
        elif (thetaTarg < -pi):
            thetaTarg = pi + (thetaTarg + pi)

        while (abs(self.theta - thetaTarg) > 0.005 and not (self._stopCurrentAction_)):
            thetaDiff = abs(self.theta - thetaTarg)

            #Set the angular velocity to optimal values that don't overshoot pi/2
            if (thetaDiff > 0.5):
                self.RobotNode_cmdvel.angular.z = 2.5 * dir
            elif (thetaDiff > 0.1):
                self.RobotNode_cmdvel.angular.z = 0.3 * dir
            else:
                self.RobotNode_cmdvel.angular.z = 0.02 * dir

            self.RobotNode_stage_pub.publish(self.RobotNode_cmdvel)

            rospy.sleep(0.0001)

            print("Rotating - current theta is " + str(self.theta) +", target theta is " + str(thetaTarg))

        if self._stopCurrentAction_ == True:
                self._stopCurrentAction_ = False
                raise ActionInterruptException.ActionInterruptException("Wall hit")
        else:
                #Stop robot by setting forward velocity to 0 and then publish change
                self.RobotNode_cmdvel.angular.z = 0
                self.RobotNode_stage_pub.publish(self.RobotNode_cmdvel)
                #return 0 for succesful finish
                return 0

    """
     @function

     @parameter:float angle

      Turn function which allows the Entity to turn a specified number of degrees either left or right.
        A negative angle would denote a right rotation and vice-versa

    """
    def face_direction(self, direction_to_face):

        current_direction = self.get_current_direction()

        print("Currently facing:" + current_direction)
        print("Turning to face: "+ direction_to_face)

        if (current_direction== direction_to_face):
            return
        elif (current_direction=="north"):
            if (direction_to_face=="east"):
                self.turn("right")
            elif(direction_to_face=="south"):
                self.rotate_relative(180, "degrees")
            elif(direction_to_face=="west"):
                self.turn("left")
            self.correct_theta()
        elif (current_direction=="east"):
            if (direction_to_face=="north"):
                self.turn("left")
            elif(direction_to_face=="south"):
                self.turn("right")
            elif(direction_to_face=="west"):
                self.rotate_relative(180,"degrees")
            self.correct_theta()
        elif (current_direction=="south"):
            if (direction_to_face=="east"):
                self.turn("left")
            elif(direction_to_face=="north"):
                self.rotate_relative(180,"degrees")
            elif(direction_to_face=="west"):
                self.turn("right")
            self.correct_theta()
        elif (current_direction=="west"):
            if (direction_to_face=="east"):
                self.rotate_relative(180,"degrees")
            elif(direction_to_face=="south"):
                self.turn("left")
            elif(direction_to_face=="north"):
                self.turn("right")
            self.correct_theta()
        else:
            print("Error: Face Direction")
            return

    """
    @function

    @parameter:double xCoord
    @parameter:double yCoord


    Goto function that moves the Entity to a specified Cartesian Coordinate. Will move only at right angles towards target coordinate.

    ie: from A to B

                           B
                           |
                           |
                           |
    A----------------------|
    """
    def goto(self, x_coord, y_coord):

        #try run the goto command
        try:
            print("Current x pos = " + str(self.px))
            print("Current y pos = " + str(self.py))

            if (abs(x_coord-self.px)<=0.3 and abs(y_coord-self.py)<=0.2 ):
                print("Already at coordinate!")
                return 0

            x_difference = x_coord - self.px
            y_difference = y_coord - self.py

            print("Xdiff" + str(x_difference))
            print("Ydiff" + str(y_difference))

            #error tolerance
            tol = 0.5

            if (x_difference<=-tol and y_difference<=-tol):
                print(1)
                if (x_difference<-tol):
                    self.face_direction("west")
                    self.move_forward(abs(x_difference))
                if (y_difference<-tol):
                    self.face_direction("south")
                    self.move_forward(abs(y_difference))
                return 0
            elif (x_difference>=tol and y_difference>=tol):
                print(2)
                if (x_difference>tol):
                    self.face_direction("east")
                    self.move_forward(abs(x_difference))
                if (y_difference>tol):
                    self.face_direction("north")
                    self.move_forward(abs(y_difference))
                return 0
            elif (x_difference>=tol and y_difference<=-tol):
                print(3)
                if (x_difference>tol):
                    self.face_direction("east")
                    self.move_forward(abs(x_difference))
                if (y_difference<-tol):
                    self.face_direction("south")
                    self.move_forward(abs(y_difference))
                return 0
            elif (x_difference<=-tol and y_difference>=tol):
                print(4)
                if (x_difference<-tol):
                    self.face_direction("west")
                    self.move_forward(abs(x_difference))
                if (y_difference>tol):
                    self.face_direction("north")
                    self.move_forward(abs(y_difference))

            if (x_difference>tol):
                print(5)
                self.face_direction("east")
                self.move_forward(abs(x_difference))
                return 0
            elif (x_difference<-tol):
                print(6)
                self.face_direction("west")
                self.move_forward(abs(x_difference))
                return 0
            if (y_difference>tol):
                print(7)
                self.face_direction("north")
                self.move_forward(abs(y_difference))
                return 0
            elif (y_difference<-tol):
                print(8)
                self.face_direction("south")
                self.move_forward(abs(y_difference))
                return 0
        except ActionInterruptException.ActionInterruptException as e:
            print(e.message)
            return 1
        finally:
            print("Arrived at destination:", self.px, self.py)

    """
    @function

    @return current_direction

    Gets the Entity's current compass direction, either north, south, east or west.

    """
    def get_current_direction(self):
        if(abs(self.theta- math.pi/2)<=0.1):
            current_direction = "north"
        elif (abs(self.theta-0)<=0.1):
            current_direction = "east"
        elif (abs(self.theta+math.pi/2)<=0.1):
            current_direction = "south"
        elif (abs(self.theta- math.pi)<=0.1 or abs(self.theta+math.pi)<=0.1):
            current_direction = "west"
        else:
            print("Current direction not one of the four cardinal directions")
            current_direction = self.correct_theta()

        return current_direction

    """
    @function

    @parameter:double xCoord
    @parameter:double yCoord


    Gets distance (as the crow flies) that the given coordinate is away from the Robot

    """
    def get_distance(self, x_coord, y_coord):

        distance = math.sqrt((x_coord - self.px)**2+(y_coord - self.py)**2)
        print("Distance from cuurent position: (%.2f,%.2f) to (%.2f,%.2f) is %.2f units" %(self.px, self.py, x_coord,y_coord,distance))
        return distance


    """
    @function

    Error correction function that detects small differences in angle from the cardinal directions (NESW)
    and rotates the Entity back to the nearest cardinal direction.

    """
    def correct_theta(self):
        current_direction="NoDirect"
        if (abs(self.theta-math.pi/2)<=0.4):
            print("North")
            self.rotate_relative(math.pi/2-self.theta,"radians")
            current_direction="north"
        elif (abs(self.theta- math.pi)<=0.4 ):
            print("west")
            self.rotate_relative(math.pi-self.theta,"radians")
            current_direction="west"
        elif (abs(self.theta+math.pi)<=0.4):
            print("west")
            self.rotate_relative(-math.pi-self.theta,"radians")
            current_direction="west"
        elif (abs(self.theta+math.pi/2)<=0.4):
            print("south")
            print("Diff" + str(math.pi/2+self.theta))
            self.rotate_relative(-math.pi/2-self.theta,"radians")
            current_direction="south"
        elif (abs(self.theta-0)<=0.4):
            print("east")
            self.rotate_relative(-self.theta,"radians")
            current_direction="east"

        return current_direction

    """
    @function

    Stop the robot
    """
    def stop(self):
        self.RobotNode_cmdvel.linear.x = 0.0


