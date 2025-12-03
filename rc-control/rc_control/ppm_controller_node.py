#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from hexapod_msgs.msg import RCCommand
from std_msgs.msg import String
import RPi.GPIO as GPIO
import time
import threading
from .controller_config import TX16SConfig

class PPMReader:
    """
    PPM (Pulse Position Modulation) signal reader class
    Reads RC channels from a single PPM input line
    """
    def __init__(self, pin, num_channels=16):
        # GPIO setup
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.IN)
        
        # PPM signal parameters
        self.SYNC_GAP = 2500        # Sync gap between frames (microseconds)
        self.MIN_CHANNEL = 800      # Minimum channel value (microseconds)
        self.MAX_CHANNEL = 2200     # Maximum channel value (microseconds)
        self.SBUS_MIN = 172         # SBUS protocol minimum value
        self.SBUS_MAX = 1811        # SBUS protocol maximum value
        
        # Channel data storage
        self.num_channels = num_channels
        self.channels = [0] * num_channels
        self.raw_channels = [0] * num_channels
        
        # Thread control
        self.running = True
        self.last_frame_time = 0
        self.frame_count = 0
        self.error_count = 0
        
        # Start reading thread
        self.thread = threading.Thread(target=self._read_ppm)
        self.thread.daemon = True
        self.thread.start()
    
    def _read_ppm(self):
        """
        Main PPM reading loop.
        Runs in a separate thread to capture PPM pulses.
        """
        last_tick = 0
        channel_index = 0

        while self.running:
            try:
                # Wait for rising edge
                GPIO.wait_for_edge(self.pin, GPIO.RISING)
                current_tick = time.time_ns() // 1000  # Convert to microseconds
                
                # Calculate pulse width
                if last_tick != 0:
                    pulse_width = current_tick - last_tick
                    
                    # Check for sync gap (new frame)
                    if pulse_width > self.SYNC_GAP:
                        channel_index = 0
                        self.frame_count += 1
                        self.last_frame_time = time.time()
                    
                    # Process channel data
                    elif channel_index < self.num_channels:
                        if self.MIN_CHANNEL <= pulse_width <= self.MAX_CHANNEL:
                            self.raw_channels[channel_index] = pulse_width
                            # Convert to SBUS range for compatibility
                            sbus_value = int((pulse_width - self.MIN_CHANNEL) * 
                                           (self.SBUS_MAX - self.SBUS_MIN) / 
                                           (self.MAX_CHANNEL - self.MIN_CHANNEL) + 
                                           self.SBUS_MIN)
                            self.channels[channel_index] = sbus_value
                            channel_index += 1
                        else:
                            self.error_count += 1
                
                last_tick = current_tick
                
            except Exception as e:
                self.error_count += 1
                time.sleep(0.001)  # Prevent CPU overload on errors
    
    def get_channel(self, channel):
        """
        Get specific channel value
        
        Args:
            channel: Channel number (0-based)
            
        Returns:
            int: Channel value in SBUS range
        """
        if 0 <= channel < self.num_channels:
            return self.channels[channel]
        return 0
    
    def get_raw_channel(self, channel):
        """
        Get raw channel value in microseconds
        
        Args:
            channel: Channel number (0-based)
            
        Returns:
            int: Channel value in microseconds
        """
        if 0 <= channel < self.num_channels:
            return self.raw_channels[channel]
        return 0
    
    def get_all_channels(self):
        """
        Get all channel values
        
        Returns:
            list: List of channel values in SBUS range
        """
        return self.channels.copy()
    
    def is_connected(self):
        """
        Check if PPM signal is present
        
        Returns:
            bool: True if signal is present
        """
        return (time.time() - self.last_frame_time) < 0.5
    
    def cleanup(self):
        """Cleanup GPIO and stop reading thread"""
        self.running = False
        self.thread.join(timeout=1.0)
        GPIO.cleanup(self.pin)

class PPMControllerNode(Node):
    """ROS2 node for reading PPM signal and publishing RC commands"""
    
    def __init__(self):
        super().__init__('ppm_controller')
        
        # Node parameters
        self.declare_parameter('ppm_pin', 17)  # Default GPIO pin for PPM
        self.declare_parameter('debug_enabled', True)
        
        # Initialize PPM reader
        try:
            self.ppm_reader = PPMReader(
                self.get_parameter('ppm_pin').value,
                num_channels=16
            )
            self.get_logger().info('PPM reader initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize PPM reader: {str(e)}')
            raise e

        # Publishers
        self.rc_publisher = self.create_publisher(
            RCCommand,
            'rc_commands',
            10
        )
        
        self.debug_publisher = self.create_publisher(
            String,
            'ppm_debug',
            10
        )

        # Create timers
        self.create_timer(0.02, self.publish_rc_data)  # 50Hz for RC data
        self.create_timer(1.0, self.publish_diagnostics)  # 1Hz for diagnostics
        
        self.get_logger().info('PPM controller node initialized')
    
    def publish_rc_data(self):
        """Publish RC commands from PPM data"""
        if not self.ppm_reader.is_connected():
            self.get_logger().warning('No PPM signal detected')
            return

        try:
            msg = RCCommand()
            
            # Map stick channels
            msg.right_stick_x = self.ppm_reader.get_channel(TX16SConfig.CHANNEL_CONFIG['right_x'])
            msg.right_stick_y = self.ppm_reader.get_channel(TX16SConfig.CHANNEL_CONFIG['right_y'])
            msg.left_stick_x = self.ppm_reader.get_channel(TX16SConfig.CHANNEL_CONFIG['left_x'])
            msg.left_stick_y = self.ppm_reader.get_channel(TX16SConfig.CHANNEL_CONFIG['left_y'])
            
            # Get switch positions
            switches = []
            for switch in ['sa', 'sb', 'sc', 'sd', 'se', 'sf']:
                switches.append(self.ppm_reader.get_channel(TX16SConfig.CHANNEL_CONFIG[switch]))
            msg.switches = switches
            
            # Speed knob value (normalized)
            msg.speed_knob = float(
                self.ppm_reader.get_channel(TX16SConfig.CHANNEL_CONFIG['s1'])
            ) / TX16SConfig.LIMITS['SBUS']['MAX']
            
            # Emergency stop (SE switch)
            msg.emergency_stop = (
                self.ppm_reader.get_channel(TX16SConfig.CHANNEL_CONFIG['se']) > 
                TX16SConfig.LIMITS['SBUS']['CENTER']
            )
            
            self.rc_publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing RC data: {str(e)}')
    
    def publish_diagnostics(self):
        """Publish diagnostic information"""
        if not self.get_parameter('debug_enabled').value:
            return
            
        try:
            msg = String()
            msg.data = (
                f"PPM Stats: Frames={self.ppm_reader.frame_count}, "
                f"Errors={self.ppm_reader.error_count}, "
                f"Connected={self.ppm_reader.is_connected()}"
            )
            self.debug_publisher.publish(msg)
            
            # Reset counters
            self.ppm_reader.frame_count = 0
            self.ppm_reader.error_count = 0
            
        except Exception as e:
            self.get_logger().error(f'Error publishing diagnostics: {str(e)}')
    
    def on_shutdown(self):
        """Cleanup when node is shutting down"""
        if hasattr(self, 'ppm_reader'):
            self.ppm_reader.cleanup()

def main(args=None):
    rclpy.init(args=args)
    
    node = PPMControllerNode()
    
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
