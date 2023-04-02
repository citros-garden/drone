import rclpy
import numpy as np
import time
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import subprocess

from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition

class OffboardControl(Node):

    def __init__(self):
        super().__init__('offboard')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(VehicleStatus,'/fmu/out/vehicle_status',self.vehicle_status_callback, qos_profile)
        self.local_position_sub_ = self.create_subscription(VehicleLocalPosition,'/fmu/out/vehicle_local_position' ,self.local_position_callback, qos_profile)

        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode',qos_profile)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint',qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command',qos_profile)
        
        timer_period = 0.02  # seconds
        self.position = None
        
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.dt = timer_period
        self.theta = 0.0
        self.radius = 10.0
        self.omega = 0.5
        self.offboard_setpoint_counter_ = 0
        self.bad_tries_to_offboard_counter_ = 0

        time.sleep(15)

        self.timer = self.create_timer(timer_period, self.cmdloop_callback)
 
    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        self.nav_state = msg.nav_state

    def local_position_callback(self, msg):
        self.position = [msg.x, msg.y, msg.z]

    def cmdloop_callback(self):

        self.get_logger().info(f"Position = {self.position[0]:.3f},{self.position[1]:.3f},{self.position[2]:.3f}")

        if self.offboard_setpoint_counter_ == 50:
            # Arm the vehicle
            self.arm()

        # offboard_control_mode needs to be paired with trajectory_setpoint
        self.publish_offboard_control_mode()

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_trajectory_setpoint_circle()
        elif self.bad_tries_to_offboard_counter_ < 15:
            self.engage_offBoard_mode()
            self.bad_tries_to_offboard_counter_ += 1
        else:
            self.get_logger().error("Couldn't get into OFFBOARD, Aborting...")
            exit()
        self.offboard_setpoint_counter_ += 1
     
    def arm(self):
        self.get_logger().info("Arm command sent")
        msg = VehicleCommand()
        msg.param1 = 1.0
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        self.publish_vehicle_command(msg)

    def disarm(self):
        self.get_logger().info('Disarm command sent')
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        self.publish_vehicle_command(msg)
    
    def land(self):
        self.get_logger().info('Land command sent')
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_NAV_LAND
        self.publish_vehicle_command(msg)
        
    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher_.publish(msg)
        
    def engage_offBoard_mode(self):
        self.get_logger().info('Offboard mode command sent')
        msg = VehicleCommand()
        msg.param1 = 1.0
        msg.param2 = 6.0
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.publish_vehicle_command(msg)

    def publish_trajectory_setpoint_circle(self):
        msg = TrajectorySetpoint()
              
        msg.position[0] = self.radius * np.cos(self.theta)
        msg.position[1] = self.radius * np.sin(self.theta)
        msg.position[2] = -5.0
        
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher_.publish(msg)
        
        self.theta = self.theta + self.omega * self.dt

    def publish_vehicle_command(self, msg):
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()