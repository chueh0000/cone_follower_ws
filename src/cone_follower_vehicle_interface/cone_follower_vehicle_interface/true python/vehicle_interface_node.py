import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from fs_msgs.msg import ControlCommand
from cone_follower_msgs.msg import VehicleStatus
import sys
import os
import time
import threading

# Add foxtronpi-pyclient to path to import FoxPi classes
# Assuming it's in the home directory as seen in directory listing
#sys.path.append(os.path.expanduser('~/foxtronpi-pyclient'))

#from doipclient import DoIPClient
#from doipclient.connectors import DoIPClientUDSConnector
#from udsoncan.client import Client
#from FoxPi_write import FoxPiWriteDID
#from FoxPi_read import FoxPiReadDID
#from common import get_uds_client

class VehicleInterfaceNode(Node):
    def __init__(self):
        super().__init__('vehicle_interface_node')

        # Declare parameters
        self.declare_parameter('doip_ip', '192.168.1.10') # Default IP
        self.declare_parameter('doip_logical_address', 0x0E00) # Default address
        self.declare_parameter('target_speed', 5.0) # Fixed speed in km/h
        self.declare_parameter('max_steering_angle', 26.0) # degrees

        self.ip = self.get_parameter('doip_ip').value
        self.logical_address = self.get_parameter('doip_logical_address').value
        self.target_speed = self.get_parameter('target_speed').value
        self.max_steering = self.get_parameter('max_steering_angle').value

        self.get_logger().info(f'Initializing Vehicle Interface with IP: {self.ip}, Target Speed: {self.target_speed} km/h')

        # APS Configuration
        self.PARK_SHIFT_VALUE = 2
        self.DRIVE_SHIFT_VALUE = 5
        self.aps_values = [0] * 14 # Index 10: APS_flg, 11: APS_Sta, 12: APS_Shift, 13: APSSpeedCMD

        # State
        self.current_steering_input = 0.0
        self.is_running = True
        self.connected = False

        # Initialize DoIP/UDS
        self.get_logger().info('ROS Debug Mode Enabled')
        self.connected = False
        '''try:
            self.doip_client = DoIPClient(self.ip, self.logical_address, protocol_version=3)
            self.uds_connection = DoIPClientUDSConnector(self.doip_client)
            self.uds_client = Client(self.uds_connection, request_timeout=4, config=get_uds_client())
            self.uds_client.open()
            
            self.writer = FoxPiWriteDID(self.uds_client)
            self.reader = FoxPiReadDID(self.uds_client)
            
            self.get_logger().info('Executing Reset Sequence...')
            self.writer.FoxPi_Reset_Sequence()
            time.sleep(1)

            self.get_logger().info('Putting Vehicle in Drive via APS...')
            # aps_values: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, DRIVE_SHIFT_VALUE, 0]
            # Based on aps_control.py: [10]=1 (APS_flg), [11]=2 (APS_Sta?), [12]=5 (Drive)
            self.aps_values[10] = 1
            self.aps_values[11] = 2
            self.aps_values[12] = self.DRIVE_SHIFT_VALUE
            self.aps_values[13] = int(self.target_speed)
            
            self.writer.FoxPi_Driving_Ctrl(self.aps_values)
            time.sleep(1)
            
            self.connected = True
            self.get_logger().info('Vehicle Interface Ready.')'''
        
        '''except Exception as e:
            self.get_logger().error(f'Failed to connect to vehicle: {e}')'''
            # We continue but commands will fail; in production we might want to exit

        # Subscribers
        self.control_sub = self.create_subscription(
            ControlCommand,
            '/control_command',
            self.control_callback,
            10)

        # Publishers
        self.status_pub = self.create_publisher(VehicleStatus, '/vehicle_status', 10)

        # Timers
        self.status_timer = self.create_timer(0.1, self.publish_status) # 10Hz
        self.control_timer = self.create_timer(0.05, self.send_control) # 20Hz

    def control_callback(self, msg: ControlCommand):
        # Map normalized steering (-1.0 to 1.0) to degrees (-max_steering to max_steering)
        # ROS: Positive is Left, Vehicle: ? 
        # Looking at pure_pursuit_node.py, it doesn't specify sign convention for vehicle.
        # Usually positive is left.
        self.current_steering_input = msg.steering * self.max_steering
        
        self.get_logger().info(
    		f'Received Control Command | '
    		f'Steering Input: {msg.steering:.2f} | '
    		f'Steering Angle: {self.current_steering_input:.2f} deg'
	)

    def send_control(self):
        '''if not self.connected:
            return'''
           
        self.get_logger().info(
    		f'[DEBUG] Sending Steering Command: '
    		f'{self.current_steering_input:.2f} deg'
	)

        '''try:
            # Update steering in aps_values if possible? 
            # Wait, FoxPi_Driving_Ctrl in FoxPi_write.py has Angle at DID_list[6]
            # DID_list[6] is Angle_Target
            # aps_control.py only updated current_speed at index 13.
            # Let's check FoxPi_write.py again for Angle_Target index.
            # Byte list construction:
            # byte_list = [
            #     ACCReq (3), ACCReq_A (1), TargetSpdReq (3), TargetSpdReq_A (1),
            #     Angle_Target_Valid (1), Angle_Target_Req (1), Angle_Target (4),
            #     ...
            # ]
            # DID_list[6] is Angle_Target.
            
            # Re-initialize aps_values with steering
            # We need to make sure we don't overwrite the APS bits
            self.aps_values[4] = 1 # Angle_Target_Valid
            self.aps_values[5] = 1 # Angle_Target_Req
            self.aps_values[6] = self.current_steering_input
            
            self.writer.FoxPi_Driving_Ctrl(self.aps_values)'''
        '''except Exception as e:
            self.get_logger().warn(f'Error sending control: {e}')'''

    def publish_status(self):
        '''if not self.connected:
            return'''

        try:
            msg = VehicleStatus()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            
            msg.speed = self.target_speed
            msg.steering_angle = self.current_steering_input
            
            self.status_pub.publish(msg)
            self.get_logger().info(
            	f'[DEBUG STATUS] '
            	f'Speed: {msg.speed:.2f} km/h | '
            	f'Steering: {msg.steering_angle:.2f} deg'
            )

            # Read DIDs
            '''motion = self.reader.FoxPi_Motion_Status()
            eps = self.reader.FoxPi_EPS_Status()
            battery = self.reader.FoxPi_Battery_Status()
            ctrl_read = self.reader.FoxPi_Driving_Ctrl()

            msg.speed = float(motion.get('VehicleSpeed', 0.0))
            msg.long_accel = float(motion.get('LongAcc', 0.0))
            msg.lat_accel = float(motion.get('LatAcc', 0.0))
            msg.yaw_rate = float(motion.get('YawRate', 0.0))
            
            msg.steering_angle = float(eps.get('SAS_Angle', 0.0))
            
            msg.battery_soc = float(battery.get('HVBattSOC', 0.0))
            msg.battery_voltage = float(battery.get('LVBatt12V', 0.0))
            
            msg.gear = int(ctrl_read.get('APS_Shift', 0))

            self.status_pub.publish(msg)'''
        except Exception as e:
            self.get_logger().warn(f'Error reading status: {e}')

      def shutdown_sequence(self):
       	self.get_logger().info('Shutting down debug interface node...')
    '''def shutdown_sequence(self):
        self.get_logger().info('Shutdown initiated. Stopping vehicle safely...')
        self.is_running = False
        
        if self.connected:
            try:
                # 1. Decelerate to 0
                target_speed = self.aps_values[13]
                while target_speed > 0:
                    target_speed -= 1
                    self.aps_values[13] = target_speed
                    self.writer.FoxPi_Driving_Ctrl(self.aps_values)
                    self.get_logger().info(f'Decelerating... Target: {target_speed} km/h')
                    time.sleep(0.5)
                
                # 2. Put in Park
                self.get_logger().info('Putting in Park...')
                self.aps_values[12] = self.PARK_SHIFT_VALUE
                self.writer.FoxPi_Driving_Ctrl(self.aps_values)
                time.sleep(1)
                
                # 3. Disable Control
                self.get_logger().info('Disabling APS Control...')
                disable_aps = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]
                self.writer.FoxPi_Driving_Ctrl(disable_aps)
                self.writer.FoxPi_Ctrl_Enable_Switch([0])
                
                self.uds_client.close()
            except Exception as e:
                self.get_logger().error(f'Error during shutdown: {e}')'''

def main(args=None):
    rclpy.init(args=args)
    node = VehicleInterfaceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_sequence()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
