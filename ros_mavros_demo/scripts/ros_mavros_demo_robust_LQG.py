# Copyright (c) 2022 hs293go
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseStamped, Vector3, Quaternion
from mavros_msgs.msg import AttitudeTarget, State
from std_msgs.msg import Header
from mavros_msgs.srv import CommandBool, SetMode
from tf.transformations import quaternion_from_euler


class WaypointController:
    """Dirt simple waypoint controller for ROS, Mavros, and PX4 simulation"""

    def __init__(self):
        # rospy.get_param(param_name)
        # private_param = rospy.get_param('~private_name')
        m = 1.3 #kg
        path = rospy.get_param("~path", [[0.0, 0.0, 10.0]])
        Aq = rospy.get_param("~Aq", [[-8.69E-05,	0,		0,		0.999985067,	0,		0,		0,		-1.732423533,	0,		0,		0,	0,	0,	0],
                                    [0,		-7.32E-05,	2.69E-21,	0,		1,		-2.45E-21,	2.20E-37,	0,		-1.732347705,	2.32E-21,	0,	0,	0,	2.37E-37],
                                    [0,		2.69E-21,	-7.33E-05,	0,		8.32E-23,	0.999999995,	-2.03E-21,	0,		-1.13E-21,	-1.73246767,	0,	0,	0,	-6.20E-21],
                                    [-10.55658757,	0,		0,		-11.72795587,	0,		0,		0,		-10291.69051,	0,		0,		-12209.75167,	0,	0,	0],
                                    [0,		-10.32516946,	5.40E-13,	0,		-9.867845914,	-3.69E-13,	-6.82E-29,	0,		-10291.69039,	-4.25E-13,	0,	-10290.69,	2.85E-13,	8.55E-29],
                                    [0,		-4.92E-14,	-0.809730567,	0,		-2.27E-15,	-0.889881781,	-1.21E-14,	0,		2.84E-14,	-807.9236757,	0,	1.75E-15,	-806.9230769,	-1.38E-14],
                                    [0,		-5.24E-30,	-5.83E-14,	0,		-7.09E-31,	-2.04E-14,	-1,		0,		8.39E-30,	2.91E-14,	0,	8.88E-31,	-2.33E-14,	-1050],
                                    [-8.69E-05,	0,		0,		-1.49E-05,	0,		0,		0,		-1.732423533,	0,		0,		1,	0,	0,	0],
                                    [0,		-7.32E-05,	2.69E-21,	0,		-3.73E-10,	-2.45E-21,	2.20E-37,	0,		-1.732347705,	2.32E-21,	0,	1,	0,	2.37E-37],
                                    [0,		2.69E-21,	-7.33E-05,	0,		8.32E-23,	-4.76E-09,	-2.03E-21,	0,		-1.13E-21,	-1.73246767,	0,	0,	1,	-6.20E-21],
                                    [-9.83E-05,	0,		0,		-8.30E-06,	0,		0,		0,		-1.000508199,	0,		0,		0,	0,	0,	0],
                                    [0,		-0.000100044,	3.56E-21,	0,		1.02E-05,	-3.26E-21,	2.83E-37,	0,		-1.000390992,	3.21E-21,	0,	0,	0,	3.30E-37],
                                    [0,		7.01E-21,	-0.000100073,	0,		1.97E-22,	0.000130013,	1.39E-21,	0,		-3.70E-21,	-1.000598756,	0,	0,	0,	-9.99E-21],
                                    [0,		0,		0,		0,		0,		0,		0,		0,		0,		0,		0,	0,	0,	-1]])
        
        Bq = rospy.get_param("~Bq", [[-1.732423533,	0,		0,		0],
                                    [0,		-1.732347705,	2.32E-21,	2.37E-37],
                                    [0,		-1.13E-21,	-1.73246767,	-6.20E-21],
                                    [-1.000508199,	0,		0,		0],
                                    [0,		-1.000390992,	3.21E-21,	3.30E-37],
                                    [0,		-3.70E-21,	-1.000598756,	-9.99E-21],
                                    [0,		0,		0,		-1],
                                    [-0.000372726,	0,		0,		0],
                                    [0,		-0.000296897,	2.32E-21,	2.37E-37],
                                    [0,		-1.13E-21,	-0.000416862,	-6.20E-21],
                                    [-0.000508199,	0,		0,		0],
                                    [0,		-0.000390992,	3.21E-21,	3.30E-37],
                                    [0,		-3.70E-21,	-0.000598756,	-9.99E-21],
                                    [0,		0,		0,		0]])

        F = rospy.get_param("~Bq", [[1,		0,		0,		1.186485228,	0,		0,		0],
	                                [0,		-1,		-4.13E-17,	0,		-1,		2.77E-17,	8.31E-33],
	                                [0,		3.52E-17,	-1,		0,		2.17E-18,	-1,		-1.71E-17],
	                                [0,		8.00E-33,	2.78E-17,	0,		8.47E-34,	-2.22E-17,	-1]])

        L = rospy.get_param("~L", [[-1.732050808,	0,		0,		0],
                                    [0,		-1.732050808,	0,		0],
                                    [0,		0,		-1.732050808,	0],
                                    [-1,		0,		0,		0],
                                    [0,		-1,		0,		0],
                                    [0,		0,		-1,		0],
                                    [0,		0,		0,		-1]])

        F_star = rospy.get_param("~F_star", [[0.076094724,	0,		0,		0.009024208,	0,		0,		0,		1050,	0,		0,		1245.80949,	0,		0,		0],
                                            [0,		-0.052504528,	5.51E-14,	0,		-0.005897666,	-3.77E-14,	-6.96E-30,	0,	-1050,		-4.34E-14,	0,		-1050,		2.91E-14,     8.72E-30],
                                            [0,		-6.39E-14,	-0.052519642,	0,		-2.95E-15,	-0.157015332,	-1.57E-14,	0,	3.70E-14,	-1050,		0,		2.28E-15,	-1050,	      -1.79E-14],
                                            [0,		-5.25E-30,	-5.83E-14,	0,		-7.10E-31,	-2.04E-14,	0,		0,	8.40E-30,	2.91E-14,	0,		8.89E-31,	-2.33E-14,	  -1050]])

        A = rospy.get_param("~A", [[0,0,0,1,0,0,0],
                                    [0,0,0,0,1,0,0],
                                    [0,0,0,0,0,1,0],
                                    [0,0,0,0,0,0,0],
                                    [0,0,0,0,0,0,0],
                                    [0,0,0,0,0,0,0],
                                    [0,0,0,0,0,0,0]])

        C2 = rospy.get_param("~C2", [[1,0,0,0,0,0,0],
                                    [0,1,0,0,0,0,0],
                                    [0,0,1,0,0,0,0],
                                    [0,0,0,0,0,0,1]])

        B2 = rospy.get_param("~B2", [[0,0,0,0],
                                    [0,0,0,0],
                                    [0,0,0,0],
                                    [-9.81,0,0,0],
                                    [0,9.81,0,0],
                                    [0,0,1/m,0],
                                    [0,0,0,1]])

        self._A = np.asarray(A)
        self._C2 = np.asarray(C2)
        self._B2 = np.asarray(B2)
        self._Aq = np.asarray(Aq)
        self._Bq = np.asarray(Bq)
        self._F = np.asarray(F)
        self._L = np.asarray(L)
        self._F_star = np.asarray(F_star)

        # numpy.asarray: convert the input to an array
        self._path = np.asarray(path)

        self._n_wps = self._path.shape[0]
        rospy.loginfo(f"Got {self._n_wps} waypoints")
        self._position = np.zeros((3,))
        self._att = AttitudeTarget()
        self._subs = {}

        # Define self.controller parameters
        self._input = np.zeros((4,))
        self._x_hat = [self._position[0], self._position[1], self._position[2], 0, 0, 0, 0]
        self._output = [self._position[0], self._position[1], self._position[2], 0] # psi=0

        # Subscribe
        self._subs["pose"] = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self._pose_cb, queue_size=1
        )
        self._subs["state"] = rospy.Subscriber(
            "/mavros/state", State, self._state_cb, queue_size=1
        )

        # Publish
        self._setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_position/local", PoseStamped, queue_size=1
        )
        self._wp_idx = 0

        # Publish attitude
        self._attitude_pub = rospy.Publisher(
            "/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=1
        )

        # Added attitude part
        self._att.body_rate = Vector3()
        self._att.header = Header()
        self._att.header.frame_id = "base_footprint"
        self._att.orientation = Quaternion(*quaternion_from_euler(0, 0, 0))
        self._att.thrust = -0.2
        self._att.type_mask = 7  # ignore body rate

        # You can optionally pass in a default value to use if the parameter is not set.
        self._roa = rospy.get_param("radius_of_acceptance", 0.5)  # type: ignore
        looptime = rospy.get_param("loop_time", 0.005)  # type: ignore

        # rospy.Timer(period, callback, oneshot=False)
        # period: This is the period between calls to the timer callback
        # callback: This is the function to be called.
        self._timer = rospy.Timer(
            period=rospy.Duration.from_sec(looptime), callback=self.main_loop
        )

        self._armed = False
        self._is_offboard = False
        self._auto_arm()
        self._auto_set_mode()

    def main_loop(self, timer_event):
        # define the current waypoint
        curr_wp = self._path[self._wp_idx]

        # ??????????????
        if not np.any(self._position):
            return

        # define the error between the desired waypoint and the current position
        delta = np.linalg.norm(curr_wp - self._position)
        # if the drone reached the waypoint
        if delta < self._roa:
            # if it is the last waypoint
            if self._wp_idx >= self._n_wps - 1:
                rospy.loginfo("Reached final waypoint! Exiting!")
                self._timer.shutdown()
            # if it is not the last waypoint
            rospy.loginfo(f"Reached waypoint {self._wp_idx}!")
            self._wp_idx += 1
        rospy.loginfo_throttle(1, f"Distance to waypoint: {delta}")

        # Controller Part
        # Read the output
        psi_command = 0
        output = [self._position[0], self._position[1], self._position[2], psi_command] #need fix: psi_real
        # Estimate the state
        y_hat = np.dot(self._C2,self._x_hat)
        self._x_hat += 0.005*(np.dot(self._A,self._x_hat)+np.dot(self._B2,self._input)+np.dot(self._L, y_hat-output)) #need check: the looptime = 0.005?
        # print([self._x_hat[0]-self._position[0], self._x_hat[1]-self._position[1], self._x_hat[2]-self._position[2]]) #print error between estimated xzy and true xyz
        # Calculate the input
        # self._input = np.dot(self._F,-self._x_hat+[curr_wp[0],curr_wp[1],curr_wp[2],0,0,0,0]) #############
        self._input = np.dot(self._F,np.asarray([self._position[0],self._position[1],self._position[2],0,0,0,0])-np.asarray([curr_wp[0],curr_wp[1],curr_wp[2],0,0,0,0]))
        # rospy.loginfo_throttle(1, f"Euler angle command: {self._input}")

        # Calculate the command
        self._att.thrust = self._input[2]
        self._att.orientation = Quaternion(*quaternion_from_euler(self._input[0], self._input[1], 0))

        # Print
        rospy.loginfo(f"current position: {[self._position[0], self._position[1], self._position[2]]}")
        rospy.loginfo(f"desired position: {[curr_wp[0], curr_wp[1], curr_wp[2]]}")
        rospy.loginfo(f"Error between position: {np.asarray([self._position[0],self._position[1],self._position[2]])-np.asarray([curr_wp[0],curr_wp[1],curr_wp[2]])}")
        # rospy.loginfo(f"Current heading: {}")


        # publish att
        # rospy.loginfo(f"The orientation is {[self._input[0], self._input[1], 0]}!")
        self._att.header.stamp = rospy.Time.now()
        self._attitude_pub.publish(self._att)

        r = rospy.Rate(200) #Hz
        try:
            r.sleep()
        except rospy.ROSInterruptException:
            pass


    # set offboard mode
    def _auto_set_mode(self):
        # call a service
        srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        try:
            # try at a rate of 1hz
            rate = rospy.Rate(1)
            # if it is not offboard
            while not self._is_offboard:
                rospy.loginfo("attempting to set OFFBOARD...")
                result = srv(custom_mode="OFFBOARD") #result???
                if result.mode_sent:
                    rospy.loginfo(f"set OFFBOARD: {result}")
                rate.sleep()
        except rospy.ServiceException as err:
            rospy.logerr(err)

    # arm
    def _auto_arm(self):
        srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        try:
            rate = rospy.Rate(1)
            while not self._armed:
                rospy.loginfo("attempting to arm...")
                result = srv(value=True)
                if result.success:
                    rospy.loginfo(f"arming: {result}")
                rate.sleep()
        except rospy.ServiceException as err:
            rospy.logerr(err)

    def _pose_cb(self, msg):
        self._position = self.comp2arr(msg.pose.position)

    def _state_cb(self, msg):
        self._armed = msg.armed
        self._is_offboard = msg.mode == "OFFBOARD"

    @staticmethod
    def comp2arr(struct):
        components = "xyzw" if hasattr(struct, "w") else "xyz"
        return np.array([getattr(struct, it) for it in components])

    @staticmethod
    def arr2comp(arr, dataclass):
        components = "xyzw" if hasattr(dataclass, "w") else "xyz"
        return dataclass(**dict(zip(components, arr)))


def main():
    rospy.init_node("ros_mavros_demo")
    _ = WaypointController()
    rospy.spin()


if __name__ == "__main__":
    main()
