#!/usr/bin/env python3

import rospy

import numpy as np

from std_msgs.msg import Empty

from crazyflie_msgs.msg import PositionVelocityStateStamped


class WaypointNode:
    def __init__(self):
        rospy.init_node("cf_waypoint")  # Initialize the ROS node

        rospy.Subscriber("in_flight", Empty, self.waypoint_update)  # Create a subscriber for the topic 'in_flight'

        self.pub = rospy.Publisher("ref", PositionVelocityStateStamped, queue_size=10)
        self.newref = PositionVelocityStateStamped()  # Create a new message object

    def waypoint_update(self, msg):
        count = 0  # Initializing indexing

        waypoint_total = rospy.get_param("~waypoints", default=[1.0, 1.0, 1.5])  # read data from launch file

        waypoint_total = eval(waypoint_total)  # change string into list

        for waypoint in waypoint_total:
            # Get the waypoint coordinates from the launch file

            waypoint = waypoint_total[count]  # read the next  waypoint location

            self.newref.header.stamp = rospy.Time.now()  # Set the header timestamp to the current time

            self.newref.state.x = waypoint[0]  # Set the x coordinate
            self.newref.state.y = waypoint[1]  # Set the y coordinate
            self.newref.state.z = waypoint[2]  # Set the z coordinate
            self.newref.state.x_dot = 0.0  # Set the x velocity
            self.newref.state.y_dot = 0.0  # Set the y velocity
            self.newref.state.z_dot = 0.0  # Set the z velocity
            self.pub.publish(self.newref)  # Publish to /ref the desired coordinate

            count = count + 1

            rospy.sleep(1)  # let the drone fly to location (wait 1 second)


if __name__ == "__main__":
    cf_waypoint_node = WaypointNode()

    rospy.spin()  # Keep the node alive
