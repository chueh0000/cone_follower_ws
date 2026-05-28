import rclpy
from rclpy.node import Node
import time
import math
from cone_follower_msgs.msg import ControlCommand

# Binary dependencies must be available in the same directory or Python path
from .FoxPi_write import FoxPiWriteDID
from .FoxPi_read import FoxPiReadDID
from .client_config import DOIP_SERVER_IP, DoIP_LOGICAL_ADDRESS
from .common import get_uds_client

from doipclient import DoIPClient
from doipclient.connectors import DoIPClientUDSConnector
from udsoncan.client import Client

class VehicleInterfaceNode(Node):
    def __init__(self):
        super().__init__('vehicle_interface_node')
        
        # Parameters
        self.declare_parameter('target_speed_kph', 1.0)
        self.target_speed = self.get_parameter('target_speed_kph').value
        
        # Constants
        self.PARK_SHIFT_VALUE = 2
        self.DRIVE_SHIFT_VALUE = 5
        self.STEERING_DELTA_LIMIT = 100.0
        
        # State
        self.current_sas_angle = 0.0
        self.steering_activated = False
        self.driving_ctrl_values = [0] * 14
        
        # Initialize DoIP/UDS Connection
        self.get_logger().info(f"Connecting to vehicle at {DOIP_SERVER_IP}...")
        try:
            self.doip_client = DoIPClient(DOIP_SERVER_IP, DoIP_LOGICAL_ADDRESS, protocol_version=3)
            self.uds_connection = DoIPClientUDSConnector(self.doip_client)
            
            # Use a context manager-like approach or manual open/close
            # We'll keep the client open for the life of the node
            self.uds_client = Client(self.uds_connection, request_timeout=4, config=get_uds_client())
            self.uds_client.open()
            
            self.fox_write = FoxPiWriteDID(self.uds_client)
            self.fox_read = FoxPiReadDID(self.uds_client)
            
            # --- 5-Step Reset Sequence ---
            self.get_logger().info("Executing Reset Sequence...")
            self.fox_write.FoxPi_Reset_Sequence()
            time.sleep(1.0)
            
            # --- Enable APS Drive Mode ---
            self.get_logger().info("Enabling APS Drive Mode...")
            # index 10: APS_flg=1, 11: APSSta=2 (Active), 12: Shift=Drive
            self.driving_ctrl_values[10] = 1
            self.driving_ctrl_values[11] = 2
            self.driving_ctrl_values[12] = self.DRIVE_SHIFT_VALUE
            self.driving_ctrl_values[13] = 0 # Initial speed 0
            self.fox_write.FoxPi_Driving_Ctrl(self.driving_ctrl_values)
            time.sleep(1.0)
            
            # --- Initial Steering Handshake ---
            self.get_logger().info("Performing Steering Activation Handshake...")
            # Step A: Valid=1, Req=0, Angle=0
            self.driving_ctrl_values[4] = 1
            self.driving_ctrl_values[5] = 0
            self.driving_ctrl_values[6] = 0
            self.fox_write.FoxPi_Driving_Ctrl(self.driving_ctrl_values)
            time.sleep(0.2)
            
            # Step B: Valid=1, Req=1, Angle=0
            self.driving_ctrl_values[5] = 1
            self.fox_write.FoxPi_Driving_Ctrl(self.driving_ctrl_values)
            time.sleep(0.2)
            
            self.steering_activated = True
            self.get_logger().info("Vehicle Interface initialized and ARMED.")
            
        except Exception as e:
            self.get_logger().error(f"Failed to connect to vehicle: {e}")
            raise e

        # Subscribers
        self.control_sub = self.create_subscription(
            ControlCommand,
            '/control_command',
            self.control_callback,
            10
        )
        
        # Periodic Status Monitoring Timer (2Hz for logging, higher for internal state if needed)
        self.status_timer = self.create_timer(0.5, self.status_callback)
        
        # Shutdown hook
        self.context.on_shutdown(self.shutdown_handler)

    def status_callback(self):
        try:
            # Read SAS Angle (DID 0x1005)
            eps_status = self.fox_read.FoxPi_EPS_Status()
            if eps_status['SAS_Angle'] != "FF":
                self.current_sas_angle = float(eps_status['SAS_Angle'])
            
            # Read Motion Status (DID 0x1002)
            motion_status = self.fox_read.FoxPi_Motion_Status()
            
            # Read Motor Status (DID 0x1010) for TqSource
            motor_status = self.fox_read.FoxPi_Motor_Status()
            
            self.get_logger().info(
                f"STATUS: Speed: {motion_status['VehicleSpeed']:.1f} kph | "
                f"SAS: {self.current_sas_angle:.1f} deg | "
                f"TqSource: {motor_status['TqSource']:.1f} "
            )
        except Exception as e:
            self.get_logger().warn(f"Failed to read status: {e}")

    def control_callback(self, msg: ControlCommand):
        if not self.steering_activated:
            return

        # 1. Map Normalized Steering (-1 to 1) to Wheel Angle (-360 to 360)
        target_wheel_angle = msg.steering * 360.0
        
        # 2. Safety Check: Steering Delta Limit
        angle_delta = abs(target_wheel_angle - self.current_sas_angle)
        if angle_delta > self.STEERING_DELTA_LIMIT:
            # Clamp or mitigate to avoid EPS dissociation
            # Here we clamp the target to be within 95 degrees of current to be safe
            self.get_logger().warn(f"Steering delta {angle_delta:.1f} > {self.STEERING_DELTA_LIMIT}. Clamping target.")
            if target_wheel_angle > self.current_sas_angle:
                target_wheel_angle = self.current_sas_angle + 95.0
            else:
                target_wheel_angle = self.current_sas_angle - 95.0

        # 3. Update driving_ctrl_values
        self.driving_ctrl_values[6] = target_wheel_angle
        self.driving_ctrl_values[13] = self.target_speed # Fixed speed as per plan
        
        # Ensure APS flags are still set
        self.driving_ctrl_values[4] = 1 # Angle_Target_Valid
        self.driving_ctrl_values[5] = 1 # Angle_Target_Req
        self.driving_ctrl_values[10] = 1 # APS_flg
        self.driving_ctrl_values[11] = 2 # APSSta Active
        self.driving_ctrl_values[12] = self.DRIVE_SHIFT_VALUE
        
        # 4. Write to Vehicle
        try:
            self.fox_write.FoxPi_Driving_Ctrl(self.driving_ctrl_values)
        except Exception as e:
            self.get_logger().error(f"Failed to send control command: {e}")

    def shutdown_handler(self):
        self.get_logger().info("Shutdown triggered. Safely stopping vehicle...")
        try:
            # 1. Set speed to 0
            self.driving_ctrl_values[13] = 0
            self.fox_write.FoxPi_Driving_Ctrl(self.driving_ctrl_values)
            time.sleep(0.5)
            
            # 2. Shift to Park
            self.driving_ctrl_values[12] = self.PARK_SHIFT_VALUE
            self.fox_write.FoxPi_Driving_Ctrl(self.driving_ctrl_values)
            time.sleep(1.0)
            
            # 3. Disable APS Control
            disable_aps = [0] * 14
            disable_aps[10] = 1
            disable_aps[11] = 0
            self.fox_write.FoxPi_Driving_Ctrl(disable_aps)
            time.sleep(0.5)
            
            # 4. Disable Enable Switch
            self.fox_write.FoxPi_Ctrl_Enable_Switch([0])
            
            self.uds_client.close()
            self.get_logger().info("Vehicle Interface safely shut down.")
        except Exception as e:
            self.get_logger().error(f"Error during shutdown: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VehicleInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown is handled by on_shutdown hook, but we can call it manually if needed
        # Or just let rclpy handle it.
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
