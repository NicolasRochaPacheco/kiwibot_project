#!/usr/bin/env python3
# =============================================================================
"""! @package location_node.py

Code Information:
    Maintainer: Nicolas Rocha Pacheco
	Mail: n.nicolas98@hotmail.com
	Candidate
"""

# =============================================================================

# Import time library
from re import S
import time

# Import rclpy
import rclpy

# Import the Node class from rclpy
from rclpy.node import Node

# Import the ReentrantCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup

# Import the Imu, NavSatFix messages definition
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

# Import the LocationMsg message definition
from usr_msgs.msg import LocationMsg


class LocationNode(Node):
    def __init__(self) -> None:
        """!
        Class constructor for LocationNode.
        """
        # Initialize super class
        super().__init__("location_node")

        # Create the callback group
        self.callback_group = ReentrantCallbackGroup()

        # Create a dictionary to store IMU data
        self.imu_data = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}

        # Subscribe to IMU data topic
        self.sub_imu_data = self.create_subscription(
            msg_type=Imu,
            topic="/imu/data",
            callback=self.cb_imu_data,
            qos_profile=5,
            callback_group=self.callback_group,
        )

        # Create a dictionary to store WiFi GPS data
        self.wifi_gps_data = {"latitude": "", "longitude": ""}

        # Define the WiFi GPS data timestamp variable
        self.wifi_gps_ts = time.time()

        # Subscribe to WiFi gps topic
        self.sub_wifi_gps_data = self.create_subscription(
            msg_type=NavSatFix,
            topic="/wifi_geo/fix",
            callback=self.cb_wifi_gps_data,
            qos_profile=5,
            callback_group=self.callback_group,
        )

        # Create a dictionary to store WiFi GPS data
        self.gps_data = {"latitude": "", "longitude": ""}

        # Subscribe to WiFi gps topic
        self.sub_gps_data = self.create_subscription(
            msg_type=NavSatFix,
            topic="/fix",
            callback=self.cb_gps_data,
            qos_profile=5,
            callback_group=self.callback_group,
        )

        # Create the publisher for LocationMsg
        self.pub_location_msg = self.create_publisher(
            msg_type=LocationMsg, topic="/custom_gps", qos_profile=5
        )

        # Create a timer to publish the LocationMsg
        self.timer_ = self.create_timer(0.1, self.cb_location_timer)

    def cb_imu_data(self, msg: Imu) -> None:
        """!
            IMU data topic callback

        @param Imu (msg): message with Imu data.
        """
        # Store the message data into member dictionary
        self.imu_data["roll"] = msg.angular_velocity.x
        self.imu_data["pitch"] = msg.angular_velocity.y
        self.imu_data["yaw"] = msg.angular_velocity.z

    def cb_wifi_gps_data(self, msg: NavSatFix) -> None:
        """! Callback for WiFi GPS data

        @param NavSatFix (msg): message with gps data.
        """
        self.wifi_gps_data["latitude"] = str(msg.latitude)
        self.wifi_gps_data["longitude"] = str(msg.longitude)
        self.wifi_gps_ts = time.time()

    def cb_gps_data(self, msg: NavSatFix) -> None:
        """! Callback for WiFi GPS data

        @param NavSatFix (msg): message with gps data.
        """
        self.gps_data["latitude"] = str(msg.latitude)
        self.gps_data["longitude"] = str(msg.longitude)

    def cb_location_timer(self) -> None:
        """!
        Timer callback to publish the LocationMsg
        """
        # Create the message instance
        msg = LocationMsg()

        # Fill the roll, pitch and yaw message fields
        msg.roll = self.imu_data["roll"]
        msg.pitch = self.imu_data["pitch"]
        msg.yaw = self.imu_data["yaw"]

        # Get current time
        _current_time = time.time()

        # Check if last WiFi GPS data has arrived within the last 20 seconds
        if _current_time - self.wifi_gps_ts < 20:
            msg.latitude = self.wifi_gps_data["latitude"]
            msg.longitude = self.wifi_gps_data["longitude"]
            msg.sensor = "wifi"
        # Fill data with bare GPS data
        else:
            msg.latitude = self.gps_data["latitude"]
            msg.longitude = self.gps_data["longitude"]
            msg.sensor = "bare"

        # Publish the message
        self.pub_location_msg.publish(msg)


def main():

    # Init
    rclpy.init()

    # Create an instance of LocationNode
    location_node_ = LocationNode()

    # Spin the node
    rclpy.spin(location_node_)

    # Gracefully destroy the node and exit
    location_node_.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
