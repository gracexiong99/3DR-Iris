# Copyright (c) 2022 hs293go
#
# This software is released under the MIT License.
# https://opensource.org/licenses/MIT

import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class WaypointController:
    """Dirt simple waypoint controller for ROS, Mavros, and PX4 simulation"""

    def __init__(self):
        # rospy.get_param(param_name)
        # private_param = rospy.get_param('~private_name')
        path = rospy.get_param("~path", [[0.0, 0.0, 10.0]])

        # numpy.asarray: convert the input to an array
        self._path = np.asarray(path)

        self._n_wps = self._path.shape[0]
        rospy.loginfo(f"Got {self._n_wps} waypoints")
        self._position = np.zeros((3,))
        self._subs = {}

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

        # publish pld
        pld = PoseStamped()
        pld.header.stamp = timer_event.current_real
        pld.pose.position = self.arr2comp(curr_wp, Point)
        self._setpoint_pub.publish(pld)

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
