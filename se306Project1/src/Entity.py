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

    global Direction, Angle


    def enum(**enums):
        return type('Enum', (), enums)

    Direction = enum(NORTH="north",EAST="east",SOUTH="south",WEST="west",LEFT="left",RIGHT="right")
    Angle = enum(DEGREES="degrees",RADIANS="radians")

    def __init__(self,r_id,x_off,y_off, theta_off):



        #declaring the instance variables
        self.robot_id = 0
        self.linearX = 2
        self.angularZ = 0

        #initial pose of the robot
        self.init_theta = theta_off
        self.init_x = x_off
        self.init_y = y_off

        self.theta = theta_off
        self.px = x_off
        self.py = y_off
        self.x_off = x_off
        self.y_off = y_off
        self.robot_id = r_id
        self.robot_node_name = ("RobotNode" +str(r_id))
        self.robot_node_identifier = ("robot_"+ str(r_id))
        self.goalx = self.px
        self.goaly = self.py

        #Used to determine how long we've waited for an element to pass by, if exceeds a threshold
        #we will know it is a static element and we need to do something different
        self.halt_counter = 0

        #array of methods of robot actions
        self._actions_ = {
            0: self.move_forward,
            1: self.goto,
            2: self.turn,
            3: self.stop,
        }

        #Enums for direction and angles

        self._actionsStack_ = []

        #stop current action used inside methods to check if there has been a call to run a new method
        self._stopCurrentAction_ = False

        #variable to track if action is running or not
        self._actionRunning_ = False

        self.disableLaser = False
        self.noMoreTrees = 0
        self.treeDetected = False
        self.atOrchard = False

        #Node Initiation
        rospy.init_node(self.robot_node_name)

        #setting up publishers and subscribers
        self.RobotNode_stage_pub = rospy.Publisher(self.robot_node_identifier+"/cmd_vel", geometry_msgs.msg.Twist, queue_size=10)
        self.StageOdo_sub = rospy.Subscriber(self.robot_node_identifier+"/odom", nav_msgs.msg.Odometry, self.StageOdom_callback)

        self.StageLaser_sub = rospy.Subscriber(self.robot_node_identifier+"/base_scan",sensor_msgs.msg.LaserScan,self.StageLaser_callback)

        self.RobotNode_cmdvel = geometry_msgs.msg.Twist()
        self.RobotNode_odom = geometry_msgs.msg.Pose2D()



    """
    @function
    @parameter: Msg msg

    Callback function to update position and other odometry values
    """
    def StageOdom_callback(self,msg):

        #Update the px and py values
        self.update_position(msg.pose.pose.position.x, msg.pose.pose.position.y)

        #Find the yaw from the quaternion values
        (roll, pitch, yaw) = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))

        #Update the theta value
        self.update_theta(yaw)

        #output_file.close("Capacity:")

        #rospy.loginfo("Current x position: %f" , self.px)
        #rospy.loginfo("Current y position: %f", self.py)
        #rospy.loginfo("Current theta: %f", self.theta)

    """
    @function
    @parameter: float x, float y

    This function is used by the StageOdom_callback function to update the current px and py values. The paremeters passed
    are the values given by the odom/msg.pose.pose.position values

    """
    def update_position(self, x, y):
        """
        This section of code calculates the absolute change in x and y positions, by combining the x and y components of
        the given x and y displacements.
        """

        #Set x_theta to the initial theta. x_theta will be used to calculate the absolute x component of the given x and y displacements
        x_theta = abs(self.init_theta)

        #Keep track of what x_theta was initially set to
        x_theta_init = x_theta

        #If the entity is initally facing towards the west, then change the x_theta value to be the difference between pi and x_theta
        if (abs(x_theta) > math.pi/2):
            x_theta = math.pi - x_theta

        #Set the y_theta variable, which is used to calculate the absolute y component of the given x and y displacements
        y_theta = math.pi/2 - self.init_theta

        #Calculate the overall change in x position by subtracting the x component of the y displacement from the x component of the
        #x displacement
        change_in_x = x * math.cos(x_theta) - y * math.cos(y_theta)

        #If the entity was initially facing westerly, then the overall change in x position will need to subtract the x component of the
        #x displacement as well
        if (x_theta_init > math.pi/2):
            change_in_x = - x * math.cos(x_theta) - y * math.cos(y_theta)

        #Calculate the overall change in y position by adding both the y component of the x and y displacements
        change_in_y = x * math.sin(x_theta) + y * math.sin(y_theta)

        #If the entity was initially facing southerly, then the y component of the x displacement will need to be subtracted
        if (self.init_theta < 0):
            change_in_y = - x * math.sin(x_theta) + y * math.sin(y_theta)

        #Update the current px and py values
        self.px = self.init_x + change_in_x
        self.py = self.init_y + change_in_y

    def update_theta(self, theta):

        #Obtain current_theta value by adding initial theta + theta value published by stage
        current_theta = self.init_theta + theta

        #If current theta exceeds value of pi, means entity is facing southwards, so update value accordingly
        if (current_theta > math.pi):
            current_theta - 2 * math.pi

        #Update the current theta vlue
        self.theta = current_theta


    """
    @function
    @parameter: int velocity

    Changes the self.linearX value to the specified velocity in m/s
    """

    def change_linear_x_to(self, velocity):
        self.linearX = velocity


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


        print "Moving Forward"
        #While the distance that the Entity has gained has not exceeded the given distance, continue to move the Entity forward
        while (dist_gained < dist and not self._stopCurrentAction_):

            #Calculate remaining distance to travel
            distToGo = dist - dist_gained

            #If the remaining distance is less than 1m, then decelerate the Entity. Having a slower moving Entity will
            #provide increased accuracy when stopping
            if (distToGo < 1):
                #Set forward velocity to 0.7m/s
                self.RobotNode_cmdvel.linear.x = 0.7
            else:
                #Set forward velocity to 2.0m/s
                self.RobotNode_cmdvel.linear.x = self.linearX


            #Publish the velocity change
            self.RobotNode_stage_pub.publish(self.RobotNode_cmdvel)

            #Sleep rospy for 2ms
            rospy.sleep(0.02)

            #Calculate the change in x and y positions from the initial x and y positions, prior to the Entity moving
            xDiff = abs(previousX - self.px)
            yDiff = abs(previousY - self.py)

            #Find the distance gained by calculating sqrt(xDiff^2 + yDiff^2)
            dist_gained = math.sqrt(xDiff * xDiff + yDiff * yDiff)

            #print("Moving Forward: " + str(distToGo) + "m to go")
            #print("Current x pos = " + str(self.px) +"," +str(self.py))


        if self._stopCurrentAction_ == True:
                #stop movement and throw exception
                self.RobotNode_cmdvel.linear.x = 0
                self.RobotNode_stage_pub.publish(self.RobotNode_cmdvel)
                self._stopCurrentAction_ = False
                raise ActionInterruptException.ActionInterruptException("Wall hit")
                #print "Move Forward: Stopped due to potential collision"
                #return 2
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
        print "Turning "+ direction
        pi = math.pi

        if (direction == Direction.LEFT):
            thetaTarg = self.theta + pi/2
            dir = 1
            if (thetaTarg > pi):
                thetaTarg = - pi + (thetaTarg - pi)
        elif (direction == Direction.RIGHT):
            thetaTarg = self.theta - pi/2
            dir = -1
            if (thetaTarg < -pi):
                thetaTarg = pi + (thetaTarg + pi)
        #disable laser as don't want to be checking for collisions when turning as
        #robot will not cause collision while turning
        self.disableLaser = True
        while (abs(self.theta - thetaTarg) > 0.01 and not self._stopCurrentAction_):
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
        #Turn complete, reenable laser
        self.disableLaser = False
        if self._stopCurrentAction_ == True:
            self._stopCurrentAction_ = False
            raise ActionInterruptException.ActionInterruptException("Wall hit")
            #return 2
        else:
            #Stop robot by setting forward velocity to 0 and then publish change
            self.RobotNode_cmdvel.angular.z = 0
            self.RobotNode_stage_pub.publish(self.RobotNode_cmdvel)
            #self.correct_theta()
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
        if (angle_type==Angle.DEGREES):
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

           # print("Rotating - current theta is " + str(self.theta) +", target theta is " + str(thetaTarg))

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
        elif (current_direction==Direction.NORTH):
            if (direction_to_face==Direction.EAST):
                self.turn(Direction.RIGHT)
            elif(direction_to_face==Direction.SOUTH):
                self.rotate_relative(180, Angle.DEGREES)
            elif(direction_to_face==Direction.WEST):
                self.turn(Direction.LEFT)
            self.correct_theta()
        elif (current_direction==Direction.EAST):
            if (direction_to_face==Direction.NORTH):
                self.turn(Direction.LEFT)
            elif(direction_to_face==Direction.SOUTH):
                self.turn(Direction.RIGHT)
            elif(direction_to_face==Direction.WEST):
                self.rotate_relative(180,Angle.DEGREES)
            self.correct_theta()
        elif (current_direction==Direction.SOUTH):
            if (direction_to_face==Direction.EAST):
                self.turn(Direction.LEFT)
            elif(direction_to_face==Direction.NORTH):
                self.rotate_relative(180,Angle.DEGREES)
            elif(direction_to_face==Direction.WEST):
                self.turn(Direction.RIGHT)
            self.correct_theta()
        elif (current_direction==Direction.WEST):
            if (direction_to_face==Direction.EAST):
                self.rotate_relative(180,Angle.DEGREES)
            elif(direction_to_face==Direction.SOUTH):
                self.turn(Direction.LEFT)
            elif(direction_to_face==Direction.NORTH):
                self.turn(Direction.RIGHT)
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
        print "Going To : ("+str(x_coord)+","+str(y_coord)+")"
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

            #If the robot needs to travel both directions
            if (not((abs(x_difference)>tol and abs(y_difference)<tol) or(abs(y_difference)>tol and abs(x_difference)<tol))):

                if (x_difference<=-tol and y_difference<=-tol):
                    if (y_difference<-tol):
                        self.face_direction(Direction.SOUTH)
                        self.move_forward(abs(y_difference))
                    if (x_difference<-tol):
                        self.face_direction(Direction.WEST)
                        self.move_forward(abs(x_difference))
                    return 0
                elif (x_difference>=tol and y_difference>=tol):
                    if (y_difference>tol):
                        self.face_direction(Direction.NORTH)
                        self.move_forward(abs(y_difference))
                    if (x_difference>tol):
                        self.face_direction(Direction.EAST)
                        self.move_forward(abs(x_difference))
                    return 0
                elif (x_difference>=tol and y_difference<=-tol):
                    if (y_difference<-tol):
                        self.face_direction(Direction.SOUTH)
                        self.move_forward(abs(y_difference))
                    if (x_difference>tol):
                        self.face_direction(Direction.EAST)
                        self.move_forward(abs(x_difference))
                    return 0
                elif (x_difference<=-tol and y_difference>=tol):
                    if (y_difference>tol):
                        self.face_direction(Direction.NORTH)
                        self.move_forward(abs(y_difference))
                    if (x_difference<-tol):
                        self.face_direction(Direction.WEST)
                        self.move_forward(abs(x_difference))
                    return 0
            #If the robot only needs to travel one direction to reach its destination
            else:
                if (x_difference>tol):
                    self.face_direction(Direction.EAST)
                    self.move_forward(abs(x_difference))
                    return 0
                elif (x_difference<-tol):
                    self.face_direction(Direction.WEST)
                    self.move_forward(abs(x_difference))
                    return 0
                if (y_difference>tol):
                    self.face_direction(Direction.NORTH)
                    self.move_forward(abs(y_difference))
                    return 0
                elif (y_difference<-tol):
                    self.face_direction(Direction.SOUTH)
                    self.move_forward(abs(y_difference))
                    return 0

        except ActionInterruptException.ActionInterruptException as e:
            print(e.message)
            return 1
        finally:

            if self._stopCurrentAction_:
                print("Halted at destination:", self.px, self.py)
                print "Go To: Stopped due to potential collision"
                return 2
            else:
                print("Arrived at destination:", self.px, self.py)
                return 0

    """
    @function

    @return current_direction

    Gets the Entity's current compass direction, either north, south, east or west.

    """
    def get_current_direction(self):
        if(abs(self.theta- math.pi/2)<=0.1):
            current_direction = Direction.NORTH
        elif (abs(self.theta-0)<=0.1):
            current_direction = Direction.EAST
        elif (abs(self.theta+math.pi/2)<=0.1):
            current_direction = Direction.SOUTH
        elif (abs(self.theta- math.pi)<=0.1 or abs(self.theta+math.pi)<=0.1):
            current_direction = Direction.WEST
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
        #print("Distance from cuurent position: (%.2f,%.2f) to (%.2f,%.2f) is %.2f units" %(self.px, self.py, x_coord,y_coord,distance))
        return distance


    """
    @function

    Error correction function that detects small differences in angle from the cardinal directions (NESW)
    and rotates the Entity back to the nearest cardinal direction.

    """
    def correct_theta(self):
        current_direction="NoDirect"
        if (abs(self.theta-math.pi/2)<=0.4):
            self.rotate_relative(math.pi/2-self.theta,Angle.RADIANS)
            current_direction=Direction.NORTH
        elif (abs(self.theta- math.pi)<=0.4 ):
            self.rotate_relative(math.pi-self.theta,Angle.RADIANS)
            current_direction=Direction.WEST
        elif (abs(self.theta+math.pi)<=0.4):
            self.rotate_relative(-math.pi-self.theta,Angle.RADIANS)
            current_direction=Direction.WEST
        elif (abs(self.theta+math.pi/2)<=0.4):
            print("Diff" + str(math.pi/2+self.theta))
            self.rotate_relative(-math.pi/2-self.theta,Angle.RADIANS)
            current_direction=Direction.SOUTH
        elif (abs(self.theta-0)<=0.4):
            self.rotate_relative(-self.theta,Angle.RADIANS)
            current_direction=Direction.EAST

        return current_direction

    """
    @function

    Stop the robot
    """
    def stop(self):
        self.RobotNode_cmdvel.linear.x = 0.0



