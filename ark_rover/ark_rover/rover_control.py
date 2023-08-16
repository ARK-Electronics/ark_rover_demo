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
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

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

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.joy_sub = self.create_subscription(
            Joy,
            '/ark_rover/joystick',
            self.joy_callback,
            10)

        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile
        )


        #Create publishers
        # self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        # self.publisher_velocity = self.create_publisher(Twist, '/fmu/in/setpoint_velocity/cmd_vel_unstamped', qos_profile)
        # self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos)

        #PX4 Overloads the ManualControlSetpoint topic so that it can distinguish between RC and Mavlink input
        self.manual_control_input_publisher_ = self.create_publisher(ManualControlSetpoint, "/fmu/in/manual_control_input", qos)
        # self.motor_control_publisher_ = self.create_publisher(ActuatorMotors, "/fmu/in/actuator_controls_0", qos_profile)

        
        #creates callback function for the arm timer
        # period is arbitrary, just should be more than 2Hz
        arm_timer_period = .05 # seconds
        self.arm_timer_ = self.create_timer(arm_timer_period, self.arm_timer_callback)
        self.throttleValue = 0.0
        self.bool1 = False
        self.bool2 = False
        self.modeVal = 0.0

        self.joy_LX = 0.0
        self.joy_LY = 0.0
        self.joy_RX = 0.0
        self.joy_RY = 0.0

        self.button_A = 0.0
        self.button_B = 0.0
        self.button_X = 0.0
        self.button_Y = 0.0
        self.button_L1 = 0.0
        self.button_L2 = 0.0

        self.D_pad_vert = 0.0
        self.D_pad_hor = 0.0

        self.trigger_L = 0.0
        self.trigger_R = 0.0

        self.gear_speed = -1.0
        self.transmission_lock = -1.0

        self.arm_one_shot = False
        self.rover_armed = False
        self.joy_stall = False
        self.throttle_moving = False
        

        self.cnt_timer = 0
        self.myCnt = 0
        self.joy_stall_timer = 0
        # self.arm_message = False
        # self.failsafe = False

        # states with corresponding callback functions that run once when state switches
        self.states = {
            "IDLE": self.state_idle,
            "ARMING": self.state_arming,
            "DRIVE": self.state_drive,
            "DISARM": self.state_disarm
        }
        self.current_state = "IDLE"
    
    # Implementation for the 'IDLE' state
    def state_idle(self):
        
        pass

    # Implementation for the 'ARMING' state
    def state_arming(self):
        self.cnt_timer = 0

    # Implementation for the 'DRIVE' state
    def state_drive(self):
    
        pass

    # Implementation for the 'DISARM' state
    def state_disarm(self):
    
        pass
    


    
    
    def arm_timer_callback(self):
        #IDLE State
        if (self.current_state == "IDLE"):
            # self.get_logger().info("INIT")
            #check if A has been pressed and released to arm
            if(self.one_shot("button_A") == True):
                self.current_state = "ARMING"
                self.cnt_timer = 0

        #ARM State
        elif(self.current_state == "ARMING"):
            # self.get_logger().info("ARMING")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            if(self.one_shot("button_A") == True):
                self.current_state = "DISARM"
            self.cnt_timer += 1
            # self.get_logger().info(f"Timer: {self.cnt_timer}")
            #Check for arm for 5 seconds
            if(self.cnt_timer >= 25):
                self.current_state = "DISARM"
            if(self.rover_armed == True):
                self.current_state = "DRIVE"

        elif(self.current_state == "DRIVE"):
            # self.get_logger().info("DRIVE")
            if(self.one_shot("button_A") == True):
                self.current_state = "DISARM"
            self.cnt_timer += 1
            if(self.rover_armed == False):
                self.current_state = "DISARM"
            
            
            self.joy_checker()

        elif(self.current_state == "DISARM"):
            # self.get_logger().info("DISARM")
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
            self.current_state = "IDLE"
                



        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1., 1.)
        
        self.publish_manual_control()
        
        
        # self.get_logger().info("Test Print")

    def one_shot(self, button_name):
        button_value = getattr(self, button_name, 0.0)

        if button_value == 1.0:
            self.arm_one_shot = True
            # self.get_logger().info(f"{button_name} pressed")
        if button_value == 0.0 and self.arm_one_shot == True:
            # self.get_logger().info("2")
            self.arm_one_shot = False
            return True
        return False

    def publish_manual_control(self):
        msg = ManualControlSetpoint()
        msg.timestamp = self.get_clock().now().nanoseconds
        msg.timestamp_sample = self.get_clock().now().nanoseconds
        msg.data_source = 2
        throttle_power = self.calculate_power(self.joy_LY, self.trigger_R, 35)
        msg.throttle = throttle_power
        msg.valid = True
        msg.sticks_moving = True
        msg.pitch = 0.0
        msg.roll = self.joy_RX
        msg.yaw = 0.0
        msg.aux1 = self.gear_speed  # -1.0 - LOW  1.0 - HIGH
        msg.aux2 = self.transmission_lock
        msg.aux3 = self.transmission_lock
        self.button_controls()
        msg.aux4 = float("NAN")
        msg.aux5 = float("NAN")
        msg.aux6 = float("NAN")
        self.manual_control_input_publisher_.publish(msg)

    def button_controls(self):
        if(not(self.throttle_moving)):
            if(self.D_pad_vert == 1.0):
                self.gear_speed = 1.0
            if(self.D_pad_vert == -1.0):
                self.gear_speed = -1.0
            if(self.D_pad_hor == -1.0):
                self.transmission_lock = -1.0
            if(self.D_pad_hor == 1.0):
                self.transmission_lock = 1.0
        
        # self.get_logger().info(f"Gear: {self.gear_speed}")

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

    def joy_callback(self, msg):
        dead_zone = .10
        self.joy_stall = False
        self.joy_stall_timer = 0
        self.joy_LX = msg.axes[0]
        self.joy_LY = -msg.axes[1] #axis needs to be inverted # rover throttle
        self.joy_RX = msg.axes[3] #rover yaw
        self.joy_RY = -msg.axes[4] #axis needs to be inverted

        self.button_A = msg.buttons[0]
        self.button_B = msg.buttons[1]
        self.button_X = msg.buttons[2]
        self.button_Y = msg.buttons[3]
        self.button_L1 = msg.buttons[4]
        self.button_L2 = msg.buttons[5]

        self.D_pad_vert = msg.axes[7]
        self.D_pad_hor = msg.axes[6]

        self.trigger_L = msg.axes[2] #trigger values go from -1 to 1
        self.trigger_R = msg.axes[5]

        if(self.joy_LY > dead_zone or self.joy_LY < -dead_zone):
            self.throttle_moving = True
        else:
            self.throttle_moving = False
        # self.get_logger().info(f"Nano: {msg.header.stamp.nanosec}")

    def joy_checker(self):
        if(self.joy_stall == True):
            self.joy_stall_timer += 1
        if(self.joy_stall_timer >= 10): #after .5 seconds stop throttle
            self.joy_LY = 0.0
        if(self.joy_stall_timer >= 100): #after 5 seconds disarm
            self.current_state = "DISARM"
        self.joy_stall = True

    def vehicle_status_callback(self, msg):
        if(msg.arming_state == 1):
            self.rover_armed = False
        else:
            self.rover_armed = True
        # self.get_logger().info(str(msg.arming_state))

    def calculate_power(self, joystick, trigger, max_power_percentage):
        # Clamp the values of joystick and trigger to the range [-1, 1]
        joystick = max(-1, min(joystick, 1))
        trigger = max(-1, min(trigger, 1))
        
        # Normalize max_power_percentage to [0, 1]
        max_power_normalized = max_power_percentage / 100

        # Calculate base power based on joystick and max_power_percentage
        base_power = joystick * max_power_normalized

        # Calculate the multiplier from the trigger value, mapping [-1, 1] to [0, 1]
        multiplier = (trigger + 1) / 2

        # Calculate final power by interpolating between base_power and 1 (100%)
        final_power = (1 - multiplier) * base_power + multiplier * joystick

        return final_power



def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()