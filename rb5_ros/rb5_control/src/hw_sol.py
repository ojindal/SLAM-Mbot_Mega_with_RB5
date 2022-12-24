#!/usr/bin/env python
import sys

# from pyrsistent import T
import rospy
from geometry_msgs.msg import Twist
from april_detection.msg import AprilTagDetectionArray
from geometry_msgs.msg import Pose2D
import numpy as np
import time
import math
scale = 0.3
"""
The class of the pid controller.
"""
class PIDcontroller:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.target = np.array([0.0,0.0,0.0])
        self.I = np.array([0.0,0.0,0.0])
        self.lastError = np.array([0.0,0.0,0.0])
        self.timestep = 0.1
        self.maximumValue = 0.1

    def setTarget(self, targetx, targety, targetw):
        """
        set the target pose.
        """
        # self.I = np.array([0.0,0.0,0.0])
        # self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array([targetx, targety, targetw])

    def setTarget(self, state):
        """
        set the target pose.
        """
        # self.I = np.array([0.0,0.0,0.0])
        # self.lastError = np.array([0.0,0.0,0.0])
        self.target = np.array(state)

    def getError(self, currentState, targetState):
        """
        return the difference between two states
        """
        result = targetState - currentState
        result[2] = (result[2] + np.pi) % (2 * np.pi) - np.pi
        return result

    def setMaximumUpdate(self, mv):
        """
        set maximum velocity for stability.
        """
        self.maximumValue = mv

    def update(self, currentState ):
        """
        calculate the update value on the state based on the error between current state and target state with PID.
        """
        e = self.getError(currentState, self.target)

        P = self.Kp * e
        self.I = self.I + self.Ki * e * self.timestep
        I = self.I
        D = self.Kd * (e - self.lastError)
        result = P + I + D

        self.lastError = e

        # scale down the twist if its norm is more than the maximum value.
        resultNorm = np.linalg.norm(result)
        if(resultNorm > self.maximumValue):
            result = (result / resultNorm) * self.maximumValue
            self.I = 0.0

        return result

def genTwistMsg(desired_twist, _u):
    """
    Convert the twist to twist msg.
    """
    _u = np.float32(_u)
    twist_msg = Twist()
    twist_msg.linear.x = desired_twist[0]
    twist_msg.linear.y = desired_twist[1]
    twist_msg.linear.z = _u[0]
    twist_msg.angular.x = _u[1]
    twist_msg.angular.y = _u[2]
    twist_msg.angular.z = desired_twist[2]
    return twist_msg

def coord(twist, current_state):
    J = np.array([[np.cos(current_state[2]), np.sin(current_state[2]), 0.0],
                  [-np.sin(current_state[2]), np.cos(current_state[2]), 0.0],
                  [0.0,0.0,1.0]])
    return np.dot(J, twist)

def updatedRobotPos(slam_cmd):
    global current_state_slam
    cap = int(slam_cmd.theta/(2*np.pi))
    _theta  = slam_cmd.theta - (2*cap*np.pi)
    # _theta  = (slam_cmd.theta / 180 ) * np.pi
    # current_state_slam = np.array([ slam_cmd.x, slam_cmd.y, slam_cmd.theta ])
    current_state_slam = np.array([ slam_cmd.x, slam_cmd.y, _theta ])
    # print('update robot pos in subs func: ', current_state_slam)

def navigate(_current_state, _target_state):

    _dt     = 0.1
    u       = np.array([0,0,0])

    if not np.array_equal(_current_state, _target_state):

        _current_state  = np.float32(_current_state)
        pid.setTarget(_target_state)

        # calculate the current twist
        update_value    = pid.update(_current_state)

        # publish the twist
        var = genTwistMsg(coord(update_value, _current_state), u)
        pub_twist.publish(var)
        # pub_twist.publish(genTwistMsg(coord(update_value, _current_state)))

        time.sleep(0.1)

        # update the current state
        _previous_state, _current_state = _current_state, _current_state + update_value
        _new_state      = _current_state

        while(np.linalg.norm(pid.getError(_current_state, _target_state)) > 0.05): # check the error between current state and current way point
            print('Error norm: ', np.linalg.norm(pid.getError(_current_state, _target_state)))
            current_state_slam = np.array([0,0,0])
            # Check for updated robot pos from SLAM
            global current_state_slam
            rospy.Subscriber('/slam', Pose2D, updatedRobotPos, queue_size=1)

            if not np.array_equal(current_state_slam, _current_state) and not np.array_equal(current_state_slam, np.array([0.0, 0.0, 0.0])):
                print('update robot pos in if cond: ', current_state_slam)
                # _current_state  = current_state_slam
                # _new_state      = _current_state

            # calculate the current twist
            update_value    = pid.update(_current_state)
            u               = (_new_state - _previous_state)/_dt

            # publish the twist
            pub_twist.publish(genTwistMsg(coord(update_value, _current_state), u))

            time.sleep(0.1)
            # update the current state
            _previous_state, _current_state = _current_state, _current_state + update_value
            _new_state      = _current_state
            # print('current state: ', _current_state)
    else:
        u       = np.array([0,0,0])
        pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0]), u))

if __name__ == "__main__":

    rospy.init_node("hw3")
    pub_twist   = rospy.Publisher("/twist", Twist, queue_size=1)

    SQUARE      = True
    OCTAGON     = False

    if SQUARE:
        waypoints = [np.array([0,0,0]), np.array([scale*2,0,0]), np.array([scale*2,scale*2,np.pi]), np.array([0,scale*2,np.pi]), np.array([0,0,0])]

    elif OCTAGON:
        waypoints = [np.array([0,0,0]), np.array([0.6,-0.6,0]), np.array([1.2,-0.6,0]), np.array([1.8,0,0]), np.array([1.8,0.6,np.pi]), np.array([1.2,1.2,np.pi]), np.array([0.6,1.2,np.pi]), np.array([0,0.6,np.pi]), np.array([0,0,0])]

    # path = np.array([])

    for i in range(len(waypoints)-1):
        # init pid controller
        pid             = PIDcontroller(0.1,0.0,0.008)
        pid.I           = np.array([0.0,0.0,0.0])
        pid.lastError   = np.array([0.0,0.0,0.0])
        current_state   = waypoints[i]
        target_state    = waypoints[i+1]
        print('Current state: {} and target state: {}'.format(current_state, target_state))
        navigate(current_state, target_state)

    # rospy.Subscriber('/apriltag_detection_array', AprilTagDetectionArray, camera_call, queue_size=1)

    # rospy.spin()
    # stop the car and exit
    u       = np.array([0,0,0])
    u       = np.float32(u)
    pub_twist.publish(genTwistMsg(np.array([0.0,0.0,0.0]), u))
    # np.savetxt("path.csv", path,delimiter =", ",fmt ='% s')
