#!/usr/bin/env python3
# =============================================================================
"""! @package plotter_node.py

Code Information:
    Maintainer: Eng. Davidson Daniel Rojas Cediel
	Mail: davidson@kiwibot.com
	Kiwi Campus / AI & Robotics Team
"""

# =============================================================================

# Basics
import time
import threading

# ROS2 dependencies
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

# ROS2 messages
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

# ROS2 Custom Messages
from usr_msgs.msg import MotorsRPM


# Plotter
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class Plotter(Node):
    def __init__(self) -> None:
        """!
        Object class constructor
        """

        super().__init__("plotter_node")

        # =============================================================================
        # Define Plotter Variables
        # =============================================================================
        self.fig, self.ax = plt.subplots(3, 1)
        self.fig.suptitle(
            "Amazing Plotter", fontsize=20, color="#E66000", fontweight=700
        )
        self.fig.set_size_inches(18.5, 10.5)
        self.fig.set_facecolor("#000000")

        for axes in self.ax:
            axes.set_facecolor("#131C26")
            axes.tick_params(color="w", labelcolor="w")
            for spine in axes.spines.values():
                spine.set_edgecolor("w")
        # =============================================================================
        # Controller Lines
        # =============================================================================

        # Linear
        (self.control_lin_ln,) = self.ax[0].plot(
            [], [], "#FF0000", label="Control Linear Signal"
        )
        (self.error_linear_ln,) = self.ax[0].plot(
            [], [], "#002BFF", label="Linear Error", linestyle="dashed"
        )
        self.controller_lin_lns = [self.control_lin_ln, self.error_linear_ln]
        self.ax[0].legend()
        self.ax[0].grid(linestyle="dashed", linewidth=0.5)
        self.lin_cmd_data, self.lin_err_data = [], []

        # Angular

        # ---------------------------------------------------------------------
        # TODO: Initialize the second subplot
        # Take care about the variables.
        # NOT define new variables use the variables defined along the code
        #
        # Self-contained reference :smile:
        (self.control_ang_ln,) = self.ax[1].plot(
            [], [], "#FF0000", label="Angular Linear Signal"
        )
        (self.error_angular_ln,) = self.ax[1].plot(
            [], [], "#002BFF", label="Angular Error", linestyle="dashed"
        )
        self.controller_ang_lns = [self.control_ang_ln, self.error_angular_ln]
        self.ax[1].legend()
        self.ax[1].grid(linestyle="dashed", linewidth=0.5)
        self.ang_cmd_data, self.ang_err_data = [], []

        #
        # End Code
        # ---------------------------------------------------------------------

        (self.fr_rpm_ln,) = self.ax[2].plot([], [], "#FF0000", label="FR RMPs")
        (self.rr_rpm_ln,) = self.ax[2].plot([], [], "#002BFF", label="RR RMPs")
        (self.rl_rpm_ln,) = self.ax[2].plot([], [], "#55FF00", label="RL RMPs")
        (self.fl_rpm_ln,) = self.ax[2].plot([], [], "#FF00EA", label="FL RMPs")
        self.rpm_lns = [self.fr_rpm_ln, self.rr_rpm_ln, self.rl_rpm_ln, self.fl_rpm_ln]
        self.ax[2].legend()
        self.ax[2].grid(linestyle="dashed", linewidth=0.5)
        self.fr_rpms_data = []
        self.rr_rpms_data = []
        self.rl_rpms_data = []
        self.fl_rpms_data = []

        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        # Define the time arrays for plots
        # +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        self.lin_cmd_time, self.lin_err_time = [], []
        self.ang_cmd_time, self.ang_err_time = [], []
        self.rpms_time = []

        # =============================================================================
        # ROS2 Stuffs
        # =============================================================================
        # Allow callbacks to be executed in parallel without restriction.
        self.callback_group = ReentrantCallbackGroup()

        # ---------------------------------------------------------------------
        # Subscribers

        self.sub_velocity_cmd = self.create_subscription(
            msg_type=Twist,
            topic="/motion_control/speed_controller/output_cmd",
            callback=self.cb_cmd_vel,
            qos_profile=5,
            callback_group=self.callback_group,
        )

        self.sub_velocity_cmd = self.create_subscription(
            msg_type=TwistStamped,
            topic="/motion_control/speed_controller/error",
            callback=self.cb_error_vel,
            qos_profile=5,
            callback_group=self.callback_group,
        )

        self.sub_rpm = self.create_subscription(
            msg_type=MotorsRPM,
            topic="/uavcan/chassis/motors_rpm_feedback",
            callback=self.cb_rpm_feedback,
            qos_profile=5,
            callback_group=self.callback_group,
        )

    # Hack Function to not block the thread
    def spin_node(self) -> None:
        """!
        Function to spin the node
        """
        rclpy.spin(self)

    # Plotter Functions
    def plot_init(self) -> None:
        """!
        Function to set the initial plot status.
        """
        self.ax[0].set_ylim(-3, 3)
        self.ax[0].set_title("Linear Signal / Linear Error", c="w")

        self.ax[1].set_ylim(-3, 3)
        self.ax[1].set_title("Angular Signal / Angular Error", c="w")

        self.ax[2].set_ylim(-170, 170)
        self.ax[2].set_title("RPMs", c="w")

        return [self.controller_lin_lns, self.controller_ang_lns]

    def update_plot(self, frame=None) -> None:
        """!
        Function to update the current figure
        """
        self.controller_lin_lns[0].set_data(self.lin_cmd_time, self.lin_cmd_data)
        self.controller_lin_lns[1].set_data(self.lin_err_time, self.lin_err_data)

        self.controller_ang_lns[0].set_data(self.ang_cmd_time, self.ang_cmd_data)
        self.controller_ang_lns[1].set_data(self.ang_err_time, self.ang_err_data)

        self.rpm_lns[0].set_data(self.rpms_time, self.fr_rpms_data)
        self.rpm_lns[1].set_data(self.rpms_time, self.rr_rpms_data)
        self.rpm_lns[2].set_data(self.rpms_time, self.rl_rpms_data)
        self.rpm_lns[3].set_data(self.rpms_time, self.fl_rpms_data)

        # Check that linear ERR time list length is not zero
        if len(self.lin_err_time) > 0:
            # Get the most recent time stamp from list
            last_lin_err_time = self.lin_err_time[-1]
            # Define the plot axes window based on most recent time
            self.ax[0].set_xlim(last_lin_err_time - 10, last_lin_err_time)

        # Check that angular ERR time list length is not zero
        if len(self.ang_err_time) > 0:
            # Get the most recent time stamp from list
            last_ang_err_time = self.ang_err_time[-1]
            # Define the plot axes window based on most recent time
            self.ax[1].set_xlim(last_ang_err_time - 10, last_ang_err_time)

        # Check that RPMs time list length is not zero
        if len(self.rpms_time) > 0:
            # Get the most recent time stamp from list
            last_rpm_time = self.rpms_time[-1]
            # Define the plot axes window based on most recent time
            self.ax[2].set_xlim(last_rpm_time - 10, last_rpm_time)

        return [self.controller_lin_lns, self.controller_ang_lns, self.rpm_lns]

    # Callback functions
    def cb_cmd_vel(self, msg: Twist) -> None:
        """!
        Callback function to get the velocity control signal.
        @param msg 'Twist' message containing the velocities of the robot
        """
        # Convert time stamp into variable
        cmd_timestamp = time.time()
        # Append time into cmd time lists
        self.lin_cmd_time.append(cmd_timestamp)
        self.ang_cmd_time.append(cmd_timestamp)
        # Append cmd values to data lists
        self.lin_cmd_data.append(msg.linear.x)
        self.ang_cmd_data.append(msg.angular.z)

    def cb_error_vel(self, msg: TwistStamped) -> None:
        """!
        Callback function to get the error between the reference and the current velocity
        @param msg 'TwistStamped' message containing the velocities of the robot
        """
        # Convert time stamp into variable
        err_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # Append time into err time lists
        self.lin_err_time.append(err_timestamp)
        self.ang_err_time.append(err_timestamp)
        # Append err values to data lists
        self.lin_err_data.append(msg.twist.linear.x)
        self.ang_err_data.append(msg.twist.angular.z)

    def cb_rpm_feedback(self, msg: MotorsRPM) -> None:
        """!
        Callback function to get motors RPMS feedback
        @param msg 'MotorsRPM' message containing the velocities of the robot
        """
        # Convert time message field to float
        rpm_timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # Append time to corresponding axis
        self.rpms_time.append(rpm_timestamp)
        # Append data into FR RPMs data variable
        self.fr_rpms_data.append(msg.rpms_fr)
        # Append data into RR RPMs data variable
        self.rr_rpms_data.append(msg.rpms_rr)
        # Append data into RL RPMs data variable
        self.rl_rpms_data.append(msg.rpms_rl)
        # Append data into FL RPMs data variable
        self.fl_rpms_data.append(msg.rpms_fl)


# =============================================================================
def main(args=None) -> None:
    """!
    Plotter Node's Main
    """

    # Initialize ROS communications for a given context.
    rclpy.init(args=args)

    # Execute work and block until the context associated with the
    # executor is shutdown.
    plotter_node = Plotter()

    # ---------------------------------------------------------------------
    # TODO: Create a Thread for spin the node
    # Use the function spin_node
    # https://realpython.com/intro-to-python-threading/

    # Create the thread using the threading library
    spin_th = threading.Thread(target=plotter_node.spin_node)

    # Start the thread
    spin_th.start()

    # ---------------------------------------------------------------------
    # End Code
    # ---------------------------------------------------------------------

    ani = FuncAnimation(
        plotter_node.fig, plotter_node.update_plot, init_func=plotter_node.plot_init
    )

    plt.show(block=True)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    plotter_node.destroy_node()
    rclpy.shutdown()


# =============================================================================
if __name__ == "__main__":
    main()
# =============================================================================
