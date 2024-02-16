#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import Empty
from crazyflie_msgs.msg import DisturbanceStamped


class DisturbanceNode:
    def __init__(self):
        rospy.init_node("disturbance")  # Initialize the ROS node

        rospy.Subscriber("in_flight", Empty, self.disturbance_update)  # Create a subscriber for the topic 'in_flight'
        disturbance_topic = rospy.get_param("~topics/disturbance")
        self.pub = rospy.Publisher(disturbance_topic, DisturbanceStamped, queue_size=1)
        self.disturbance = DisturbanceStamped()  # Create a new message object
        self.disturbance_sampler = np.random.RandomState(0)

    def disturbance_update(self, msg):
        while not rospy.is_shutdown():
            offset_y = 2 * self.disturbance_sampler.rand() - 1
            self.disturbance.header.stamp = rospy.Time.now()  # Set the header timestamp to the current time
            self.disturbance.disturbance.x = 0.0  # Set the x disturbance
            self.disturbance.disturbance.y = offset_y  # Set the y disturbance
            self.disturbance.disturbance.z = 0.0  # Set the z disturbance
            self.disturbance.disturbance.x_dot = 0.0  # Set the x velocity disturbance
            self.disturbance.disturbance.y_dot = 0.0 # Set the y velocity disturbance
            self.disturbance.disturbance.z_dot = 0.0 # Set the z velocity disturbance
            self.disturbance.disturbance.yaw = 0.0 # Set the yaw disturbance
            self.pub.publish(self.disturbance)  # Publish to /disturbance the desired disturbance
            rospy.sleep(0.1)


if __name__ == "__main__":
    disturbance_node = DisturbanceNode()

    rospy.spin()  # Keep the node alive
