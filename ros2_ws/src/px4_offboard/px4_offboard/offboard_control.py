import rclpy
import numpy as np
import time
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from enum import Enum

from std_msgs.msg import String
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleControlMode

class State(Enum):
    P1 = 1
    P2 = 2
    P3 = 3
    P4 = 4
    LAND = 5

class OffboardControl(Node):

    def __init__(self):
        super().__init__('offboard_control')

        qos_profile = self.create_qos_profile()

        self.initialize_publishers_and_subscribers(qos_profile)
        self.initialize_parameters()
        self.initialize_variables()

        self.timer = self.create_timer(self.timer_period, self.step)

    @staticmethod
    def create_qos_profile():
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT ,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        return qos_profile

    def initialize_publishers_and_subscribers(self, qos_profile):
        self.status_sub = self.create_subscription(VehicleStatus,'/fmu/out/vehicle_status',self.vehicle_status_callback, qos_profile)
        self.local_position_sub_ = self.create_subscription(VehicleLocalPosition,'/fmu/out/vehicle_local_position' ,self.local_position_callback, qos_profile)
        self.control_mode_sub = self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.control_mode_callback, qos_profile)

        self.offboard_control_mode_publisher_ = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode',qos_profile)
        self.trajectory_setpoint_publisher_ = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint',qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command',qos_profile)
        self.state_publisher = self.create_publisher(String, '/offboard/state', qos_profile)

    def initialize_parameters(self):
        self.declare_parameters(
            namespace='',
            parameters=[('p1_x', 0.0), ('p1_y', 0.0), ('p1_z', -30.0),
                        ('p2_x', 10.0), ('p2_y', 0.0), ('p2_z', -10.0),
                        ('p3_x', 10.0), ('p3_y', 10.0), ('p3_z', -10.0),
                        ('p4_x', 0.0), ('p4_y', 10.0), ('p4_z', -10.0),
                        ('repeats', 2), ('tolerance', 0.5), ('timer_period', 0.02),]
        )
        self.setpoints = {"P1": [self.get_parameter('p1_x').get_parameter_value().double_value, self.get_parameter('p1_y').get_parameter_value().double_value, self.get_parameter('p1_z').get_parameter_value().double_value],
                          "P2": [self.get_parameter('p2_x').get_parameter_value().double_value, self.get_parameter('p2_y').get_parameter_value().double_value, self.get_parameter('p2_z').get_parameter_value().double_value],
                          "P3": [self.get_parameter('p3_x').get_parameter_value().double_value, self.get_parameter('p3_y').get_parameter_value().double_value, self.get_parameter('p3_z').get_parameter_value().double_value],
                          "P4": [self.get_parameter('p4_x').get_parameter_value().double_value, self.get_parameter('p4_y').get_parameter_value().double_value, self.get_parameter('p4_z').get_parameter_value().double_value],
                          "LAND": [0.0, 0.0, 0.0]}
        self.timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        self.tolerance = self.get_parameter('tolerance').get_parameter_value().double_value
        self.repeats = self.get_parameter('repeats').get_parameter_value().double_value

    def initialize_variables(self):
        self.position = None
        self.arming_state = False
        self.state = State.P1
        self.target_msg = TrajectorySetpoint()
        self.offboard_state_msg = String()
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.repeats_counter = 0
        self.bad_tries_to_offboard_counter_ = 0
        self.maximum_number_of_offboard_tries = 15

    def vehicle_status_callback(self, msg):
        self.nav_state = msg.nav_state
        
    def control_mode_callback(self, msg):
        self.arming_state = msg.flag_armed
        self.flag_control_offboard_enabled = msg.flag_control_offboard_enabled

    def local_position_callback(self, msg):
        self.position = [msg.x, msg.y, msg.z]
        
    def arm(self):
        self.get_logger().info("Arm command sent", throttle_duration_sec=1.0)
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

    def publish_vehicle_command(self, msg):
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)
        
    def engage_offboard_mode(self):
        self.get_logger().info('Offboard mode command sent', throttle_duration_sec=1.0)
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
        y = self.position[1]
        z = self.position[2]

        current_position = [x, y, z]
        current_setpoint = self.setpoints[self.state.name]

        distance = np.linalg.norm([abs(p1) - abs(p2) for p1,p2 in zip(current_position, current_setpoint)])

        if distance < self.tolerance and self.state == State.P1:
            self.state = State.P2
            return
        if distance < self.tolerance and self.state == State.P2:
            self.state = State.P3
            return
        if distance < self.tolerance and self.state == State.P3:
            self.state = State.P4
            return
        if distance < self.tolerance and self.state == State.P4:
            self.state = State.P1
            self.repeats_counter += 1
            if self.repeats_counter == self.repeats:
                self.state = State.LAND
                self.get_logger().info(f"Finished Offboard, Landing!")

        if self.state == State.LAND:
            self.land()
            time.sleep(1.0)
            exit()
        
        self.target_msg.position = current_setpoint
        self.target_msg.timestamp = int(Clock().now().nanoseconds / 1000)

        self.offboard_state_msg.data = self.state.name

        self.trajectory_setpoint_publisher_.publish(self.target_msg)
        self.state_publisher.publish(self.offboard_state_msg)

        self.get_logger().info(f"State = {self.state.name}, Position = [{x:.3f}, {y:.3f}, {z:.3f}]", throttle_duration_sec=0.5)

    def step(self):
        # Arm the vehicle
        if not self.arming_state:
            self.arm()
            return

        # offboard_control_mode needs to be paired with trajectory_setpoint
        self.publish_offboard_control_mode()

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.flow()

        elif self.bad_tries_to_offboard_counter_ < self.maximum_number_of_offboard_tries:
            self.engage_offboard_mode()
            self.bad_tries_to_offboard_counter_ += 1
        else:
            self.get_logger().error("Couldn't get into OFFBOARD, Aborting...")
            exit()

def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()