#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from hexapod_msgs.msg import RCCommand
from std_msgs.msg import String
import serial
import struct
import time
from .controller_config import TX16SConfig

class SBUSControllerNode(Node):
    def __init__(self):
        super().__init__('sbus_controller')
        
        # Node parameters
        self.declare_parameter('uart_port', '/dev/ttyAMA0')
        self.declare_parameter('uart_baudrate', 100000)
        
        # SBUS protocol constants
        self.SBUS_HEADER = 0x0F
        self.SBUS_FOOTER = 0x00
        self.SBUS_FRAME_LENGTH = 25  # 1 header + 22 data + 2 footer bytes
        self.SBUS_CHANNELS = 16      # Number of available channels
        
        # Initialize UART for SBUS
        try:
            self.uart = serial.Serial(
                port=self.get_parameter('uart_port').value,
                baudrate=self.get_parameter('uart_baudrate').value,
                parity=serial.PARITY_EVEN,
                stopbits=serial.STOPBITS_TWO,
                bytesize=serial.EIGHTBITS
            )
            self.get_logger().info('SBUS UART initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize SBUS UART: {str(e)}')
            raise e

        # Initialize publishers
        self.rc_publisher = self.create_publisher(
            RCCommand,
            'rc_commands',
            10
        )
        
        self.debug_publisher = self.create_publisher(
            String,
            'sbus_debug',
            10
        )

        # Variables for frame processing
        self.frame_buffer = bytearray()
        self.channel_values = [0] * self.SBUS_CHANNELS
        self.last_frame_time = time.time()
        self.frame_loss_count = 0
        
        # Create timer for reading SBUS data
        self.create_timer(0.004, self.read_sbus)  # 250Hz timer for SBUS reading
        
        # Status monitoring timer
        self.create_timer(1.0, self.check_connection_status)
        
        self.get_logger().info('SBUS controller node initialized')

    def read_sbus(self):
        """Read and process SBUS frames from UART"""
        try:
            if self.uart.in_waiting:
                # Read available bytes
                data = self.uart.read(self.uart.in_waiting)
                self.frame_buffer.extend(data)
                
                # Process complete frames
                while len(self.frame_buffer) >= self.SBUS_FRAME_LENGTH:
                    # Look for frame header
                    if self.frame_buffer[0] != self.SBUS_HEADER:
                        self.frame_buffer.pop(0)
                        continue
                    
                    # Check if we have a complete frame
                    if len(self.frame_buffer) >= self.SBUS_FRAME_LENGTH:
                        frame = self.frame_buffer[:self.SBUS_FRAME_LENGTH]
                        self.frame_buffer = self.frame_buffer[self.SBUS_FRAME_LENGTH:]
                        
                        # Process the frame
                        if self.process_frame(frame):
                            self.publish_rc_data()
                            self.last_frame_time = time.time()
                        else:
                            self.frame_loss_count += 1
                            self.get_logger().warning('Invalid SBUS frame received')
                
        except Exception as e:
            self.get_logger().error(f'Error reading SBUS data: {str(e)}')

    def process_frame(self, frame):
        """
        Process SBUS frame and extract channel values
        
        Args:
            frame: Raw SBUS frame bytes
            
        Returns:
            bool: True if frame was processed successfully
        """
        # Verify frame header and footer
        if frame[0] != self.SBUS_HEADER or frame[-1] != self.SBUS_FOOTER:
            return False
        
        # Extract channel data
        try:
            # SBUS protocol uses 11 bits per channel, packed across bytes
            data = struct.unpack('B' * 23, frame[0:23])
            
            # Decode channels
            self.channel_values[0]  = ((data[1]     | data[2] << 8)                     & 0x07FF)
            self.channel_values[1]  = ((data[2] >> 3 | data[3] << 5)                    & 0x07FF)
            self.channel_values[2]  = ((data[3] >> 6 | data[4] << 2 | data[5] << 10)    & 0x07FF)
            self.channel_values[3]  = ((data[5] >> 1 | data[6] << 7)                    & 0x07FF)
            self.channel_values[4]  = ((data[6] >> 4 | data[7] << 4)                    & 0x07FF)
            self.channel_values[5]  = ((data[7] >> 7 | data[8] << 1 | data[9] << 9)     & 0x07FF)
            self.channel_values[6]  = ((data[9] >> 2 | data[10] << 6)                   & 0x07FF)
            self.channel_values[7]  = ((data[10] >> 5 | data[11] << 3)                  & 0x07FF)
            self.channel_values[8]  = ((data[12]     | data[13] << 8)                   & 0x07FF)
            self.channel_values[9]  = ((data[13] >> 3 | data[14] << 5)                  & 0x07FF)
            self.channel_values[10] = ((data[14] >> 6 | data[15] << 2 | data[16] << 10) & 0x07FF)
            self.channel_values[11] = ((data[16] >> 1 | data[17] << 7)                  & 0x07FF)
            self.channel_values[12] = ((data[17] >> 4 | data[18] << 4)                  & 0x07FF)
            self.channel_values[13] = ((data[18] >> 7 | data[19] << 1 | data[20] << 9)  & 0x07FF)
            self.channel_values[14] = ((data[20] >> 2 | data[21] << 6)                  & 0x07FF)
            self.channel_values[15] = ((data[21] >> 5 | data[22] << 3)                  & 0x07FF)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error decoding SBUS frame: {str(e)}')
            return False

    def publish_rc_data(self):
        """Publish processed RC commands"""
        try:
            msg = RCCommand()
            
            # Map stick channels according to TX16S configuration
            msg.right_stick_x = self.channel_values[TX16SConfig.CHANNEL_CONFIG['right_x']]
            msg.right_stick_y = self.channel_values[TX16SConfig.CHANNEL_CONFIG['right_y']]
            msg.left_stick_x = self.channel_values[TX16SConfig.CHANNEL_CONFIG['left_x']]
            msg.left_stick_y = self.channel_values[TX16SConfig.CHANNEL_CONFIG['left_y']]
            
            # Get switch positions
            switches = []
            for switch in ['sa', 'sb', 'sc', 'sd', 'se', 'sf']:
                switches.append(self.channel_values[TX16SConfig.CHANNEL_CONFIG[switch]])
            msg.switches = switches
            
            # Speed knob value (normalized)
            msg.speed_knob = float(
                self.channel_values[TX16SConfig.CHANNEL_CONFIG['s1']]
            ) / TX16SConfig.LIMITS['SBUS']['MAX']
            
            # Emergency stop (SE switch)
            msg.emergency_stop = (
                self.channel_values[TX16SConfig.CHANNEL_CONFIG['se']] > 
                TX16SConfig.LIMITS['SBUS']['CENTER']
            )
            
            self.rc_publisher.publish(msg)
            
            # Publish debug information
            debug_msg = String()
            debug_msg.data = f"Frame OK, Ch1: {self.channel_values[0]}, Ch2: {self.channel_values[1]}"
            self.debug_publisher.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing RC data: {str(e)}')

    def check_connection_status(self):
        """Monitor connection status and frame loss"""
        current_time = time.time()
        if current_time - self.last_frame_time > 1.0:
            self.get_logger().warning('No SBUS frames received for 1 second')
        
        if self.frame_loss_count > 0:
            self.get_logger().info(f'Frame loss count: {self.frame_loss_count}')
            self.frame_loss_count = 0

    def on_shutdown(self):
        """Cleanup when node is shutting down"""
        if hasattr(self, 'uart'):
            self.uart.close()

def main(args=None):
    rclpy.init(args=args)
    
    node = SBUSControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
