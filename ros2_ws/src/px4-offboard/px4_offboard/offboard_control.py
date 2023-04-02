import rclpy
import numpy as np
import time
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import subprocess
from enum import Enum
from std_msgs.msg import String
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition

class State(Enum):
    P1 = 1
    P2 = 2
    P3 = 3
    P4 = 4
    LAND = 5

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
        self.state_publisher = self.create_publisher(String, '/offboard/state', 1)
        timer_period = 0.02  # seconds

        self.position = None
        self.state = State.P1

        self.setpoints = {"P1": [0.0, 0.0, -5.0],
                          "P2": [10.0, 0.0, -5.0],
                          "P3": [10.0, 10.0, -5.0],
                          "P4": [0.0, 10.0, -5.0],
                          "LAND": [0.0, 0.0, 0.0]}
        
        self.position_msg = TrajectorySetpoint()
        self.offboard_state_msg = String()
        
        self.tolerance = 1.0
        self.repeats = 1
        self.repeats_counter = 0
        
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.dt = timer_period
        
        self.offboard_setpoint_counter_ = 0
        self.bad_tries_to_offboard_counter_ = 0

        time.sleep(15)

        self.timer = self.create_timer(timer_period, self.step)
 
    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        self.nav_state = msg.nav_state

    def local_position_callback(self, msg):
        self.position = [msg.x, msg.y, msg.z]

    def step(self):
        # Arm the vehicle
        if self.offboard_setpoint_counter_ == 50:
            self.arm()
            self.offboard_setpoint_counter_ += 1
        elif self.offboard_setpoint_counter_ < 50:
            self.offboard_setpoint_counter_ += 1

        # offboard_control_mode needs to be paired with trajectory_setpoint
        self.publish_offboard_control_mode()

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.flow()

        elif self.bad_tries_to_offboard_counter_ < 15:
            self.engage_offBoard_mode()
            self.bad_tries_to_offboard_counter_ += 1
        else:
            self.get_logger().error("Couldn't get into OFFBOARD, Aborting...")
            exit()
        
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

    def flow(self):

        x = self.position[0]
        y = self.position[0]
        z = self.position[0]

        current_position = [x, y, z]
        current_setpoint = self.setpoints[self.state.name]

        distance = np.linalg.norm([abs(p1) - abs(p2) for p1,p2 in zip(current_position, current_setpoint)])

        self.get_logger().info(f"distance = {distance}", throttle_duration_sec=0.25)

        if distance < self.tolerance and self.state == State.P1:
            self.state = State.P2
        if distance < self.tolerance and self.state == State.P2:
            self.state = State.P3
        if distance < self.tolerance and self.state == State.P3:
            self.state = State.P4
        if distance < self.tolerance and self.state == State.P4:
            self.state = State.P1
            self.repeats_counter += 1

        if self.repeats_counter == self.repeats:
            self.land()
            time.sleep(10)
            exit()
        
        self.position_msg.position[0] = current_setpoint[0]
        self.position_msg.position[1] = current_setpoint[1]
        self.position_msg.position[2] = current_setpoint[2]
        self.position_msg.timestamp = int(Clock().now().nanoseconds / 1000)

        self.offboard_state_msg.data = self.state.name

        self.trajectory_setpoint_publisher_.publish(self.position_msg)
        self.state_publisher.publish(self.offboard_state_msg)

        self.get_logger().info(f"State = {self.state.name}", throttle_duration_sec=1.0)

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