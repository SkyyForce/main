__author__ = "eugene"

import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, GotoSetpoint, VehicleControlMode, VehicleCommand, VehicleLocalPosition
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np
import math

pointers = []
indx = 0
class OffboardSetpoint(Node):

    def __init__(self):
        super().__init__('offboard_setpoint')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )


        # Publishers and Subscribers
        self.control_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(GotoSetpoint, '/fmu/in/goto_setpoint', 10)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.vehicle_local_position_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)

        self.mode_sub = self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.mode_callback, qos_profile)
        #self.subscription = self.create_subscription(data, 'aruco_coords', 10)

        # Set publish rate timer
        pub_rate = 10.0  # Hz. Set rate of > 2Hz for OffboardControlMode 
        self.timer = self.create_timer(1/pub_rate, self.setpoint_callback)

        # Other parameters
        self.offboard_mode = False
        self.vehicle_local_position = VehicleLocalPosition()

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = vehicle_local_position    

    def mode_callback(self, msg):
        self.offboard_mode = msg.flag_control_offboard_enabled

    def setpoint_callback(self):
        # info: https://docs.px4.io/main/en/flight_modes/offboard.html

        # Set and publish control flags
        control_mode = OffboardControlMode()
        # Timestamp is automatically set inside PX4
        control_mode.timestamp = 0
        # First field that has a non-zero value (from top to bottom)
        # defines what valid estimate is required
        control_mode.position = True
        control_mode.velocity = False
        control_mode.acceleration  = False
        control_mode.attitude = False
        control_mode.body_rate = False
        control_mode.thrust_and_torque = False
        control_mode.direct_actuator = False

        self.control_pub.publish(control_mode)


        # Start sending setpoints if in offboard mode
        if self.offboard_mode:
            # Trajectory setpoint - NED local world frame
            goto_set = GotoSetpoint()
            # Timestamp is automatically set inside PX4
            goto_set.timestamp = 0

            global indx
            x = pointers[indx][0]
            y = pointers[indx][1]
            z = pointers[indx][2]

            self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")
            self.get_logger().info(f"Publishing position setpoints {[self.vehicle_local_position.x, self.vehicle_local_position.y, self.vehicle_local_position.z]}")
            if math.sqrt((self.vehicle_local_position.x - x)**2 + (self.vehicle_local_position.y - y)**2 + (self.vehicle_local_position.z - z)**2) <= 0.3:
                if indx < len(pointers)-1:
                    indx = indx+1
            goto_set.position = [x, y, z] # [x, y, z] in meters

            # Publish
            self.setpoint_pub.publish(goto_set)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)


def main(args=None):
    with open('/home/eugene/ros2_ws/src/aruco/resource/coords', 'r') as f:
        for line in f:
            line = line.replace('\n', '').split(' ')
            for i in range(len(line)):
                line[i] = float(line[i])

            pointers.append(line)
    f.close()


    print(pointers)
    rclpy.init(args=args)

    control = OffboardSetpoint()

    rclpy.spin(control)

    control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    
    print('hello')

    main()
    