#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

__author__ = "Braden Wagstaff"
__contact__ = "braden@arkelectron.com"

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, UXR_DURABILITY_VOLATILE, UXR_RELIABILITY_BEST_EFFORT, UXR_HISTORY_KEEP_LAST

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import ManualControlSetpoint
from px4_msgs.msg import ActuatorMotors
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Joy
from math import pi
from std_msgs.msg import Bool


class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        # qos_profile = QoSProfile(
        #     reliability=ReliabilityPolicy.BEST_EFFORT,
        #     durability=DurabilityPolicy.TRANSIENT_LOCAL,
        #     history=HistoryPolicy.KEEP_LAST,
        #     depth=1
        # )

        # qos = QoSProfile(
        # durability = UXR_DURABILITY_VOLATILE,
        # reliability = UXR_RELIABILITY_BEST_EFFORT,
        # history = UXR_HISTORY_KEEP_LAST,
        # depth = 1
        # )

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        #Create subscriptions
        # self.status_sub = self.create_subscription(
        #     VehicleStatus,
        #     '/fmu/out/vehicle_status',
        #     self.vehicle_status_callback,
        #     qos_profile)
        
        # self.offboard_velocity_sub = self.create_subscription(
        #     Twist,
        #     '/offboard_velocity_cmd',
        #     self.offboard_velocity_callback,
        #     qos_profile)
        
        # self.attitude_sub = self.create_subscription(
        #     VehicleAttitude,
        #     '/fmu/out/vehicle_attitude',
        #     self.attitude_callback,
        #     qos_profile)
        
        # self.my_bool_sub = self.create_subscription(
        #     Bool,
        #     '/arm_message',
        #     self.arm_message_callback,
        #     qos_profile)

        self.joy_sub = self.create_subscription(
            Joy,
            '/joystick',
            self.joy_callback,
            10)


        #Create publishers
        # self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        # self.publisher_velocity = self.create_publisher(Twist, '/fmu/in/setpoint_velocity/cmd_vel_unstamped', qos_profile)
        # self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos)
        self.manual_control_publisher_ = self.create_publisher(ManualControlSetpoint, "/manual_control_setpoint", qos)
        # self.motor_control_publisher_ = self.create_publisher(ActuatorMotors, "/fmu/in/actuator_controls_0", qos_profile)

        
        #creates callback function for the arm timer
        # period is arbitrary, just should be more than 2Hz
        arm_timer_period = .02 # seconds
        self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback)
        self.throttleValue = 0.0
        self.bool1 = False
        self.bool2 = False
        self.modeVal = 0.0

        

        
        self.myCnt = 0
        # self.arm_message = False
        # self.failsafe = False

        #states with corresponding callback functions that run once when state switches
        # self.states = {
        #     "IDLE": self.state_init,
        #     "ARMING": self.state_arming,
        #     "TAKEOFF": self.state_takeoff,
        #     "LOITER": self.state_loiter,
        #     "OFFBOARD": self.state_offboard
        # }
        # self.current_state = "IDLE"
    

    

    #callback function that arms, takes off, and switches to offboard mode
    #implements a finite state machine
    def arm_timer_callback(self):
        self.myCnt += 1
        # self.get_logger().info(f"Timer Count: {self.myCnt}")
        if(self.myCnt > 150):
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            # self.get_logger().info("Arm command send")
        if(self.myCnt > 150):
            # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 1.)
            if(self.button1 == 1):
                self.bool1 = True
            if(self.button1 == 0 and self.bool1 == True):
                self.bool1 = False
                self.modeVal += 1.0
            # self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 1.)
            self.get_logger().info(f"ModeVal: {self.modeVal}")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 1.)
        if(self.myCnt > 150):
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 1.)
            msg = ManualControlSetpoint()
            msg.timestamp = self.get_clock().now().nanoseconds
            msg.timestamp_sample = self.get_clock().now().nanoseconds
            msg.data_source = 1
            msg.throttle = self.throttleValue
            msg.valid = True
            msg.sticks_moving = True
            msg.pitch = 0.0
            msg.roll = 0.0
            msg.yaw = 0.0
            msg.aux1 = 0.0
            msg.aux2 = 0.0
            msg.aux3 = 0.0
            msg.aux4 = 0.0
            msg.aux5 = 0.0
            msg.aux6 = 0.0
            self.manual_control_publisher_.publish(msg)

    def joy_callback(self, msg):
        self.throttleValue = msg.axes[1]
        self.yawValue = msg.axes[3]
        self.button_A = msg.buttons[0]
        self.button_B = msg.buttons[1]
        self.button_X = msg.buttons[2]
        self.button_Y = msg.buttons[3]
        self.button_L1 = msg.buttons[4]
        self.button_L2 = msg.buttons[5]

        # self.get_logger().info(f"Axes[1]: {self.throttleValue}")

    


    

        

    

    # publishes command to /fmu/in/vehicle_command
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        # msg.flag_control_manual_enabled = True
        # msg.flag_control_attitude_enabled = True
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7    # altitude value in takeoff command
        msg.command = command  # command ID
        msg.target_system = 1  # system which should execute the command
        msg.target_component = 1  # component which should execute the command, 0 for all components
        msg.source_system = 1  # system sending the command
        msg.source_component = 1  # component sending the command
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)

    

    

    


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()