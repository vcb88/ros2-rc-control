#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from hexapod_msgs.msg import RCCommand
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from .controller_config import TX16SConfig

class JoystickControllerNode(Node):
    """
    ROS2 node for handling joystick input and converting it to RC-like commands
    for hexapod control. Supports various joystick types with configurable mapping.
    """
    
    def __init__(self):
        super().__init__('joystick_controller')
        
        # Node parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('joy_deadzone', 0.05),      # Deadzone for analog sticks
                ('joy_expo', 0.3),           # Exponential curve for stick response
                ('button_mode', 'hold'),     # 'hold' or 'toggle' for button behavior
                ('debug_enabled', True)
            ]
        )
        
        # Joystick button mapping (for common controllers)
        self.BUTTON_MAP = {
            # PS4/PS5 controller mapping
            'ps': {
                'emergency_stop': 'OPTIONS',     # Options button
                'gait_mode': 'L1',              # Left shoulder button
                'speed_mode': 'R1',             # Right shoulder button
                'special_move': 'TRIANGLE',      # Triangle button
                'stance_mode': 'SQUARE',         # Square button
                'demo_mode': 'CIRCLE'            # Circle button
            },
            # Xbox controller mapping
            'xbox': {
                'emergency_stop': 'START',       # Start button
                'gait_mode': 'LB',              # Left bumper
                'speed_mode': 'RB',             # Right bumper
                'special_move': 'Y',            # Y button
                'stance_mode': 'X',             # X button
                'demo_mode': 'B'                # B button
            }
        }
        
        # Default button indices (PS4/PS5 layout)
        self.button_indices = {
            'emergency_stop': 9,   # Options button
            'gait_mode': 4,        # L1
            'speed_mode': 5,       # R1
            'special_move': 3,     # Triangle
            'stance_mode': 0,      # Square
            'demo_mode': 1,        # Circle
        }
        
        # Axis indices (common for most controllers)
        self.axis_indices = {
            'right_x': 3,   # Right stick X
            'right_y': 4,   # Right stick Y
            'left_x': 0,    # Left stick X
            'left_y': 1,    # Left stick Y
            'l2': 2,        # L2 trigger (if analog)
            'r2': 5         # R2 trigger (if analog)
        }
        
        # State variables
        self.emergency_stop = False
        self.current_gait = 'normal'
        self.current_speed = 1.0
        self.button_states = {}
        self.last_joy_time = self.get_clock().now()
        
        # Subscribe to joystick messages
        self.joy_subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )
        
        # Publishers
        self.rc_publisher = self.create_publisher(
            RCCommand,
            'rc_commands',
            10
        )
        
        self.debug_publisher = self.create_publisher(
            String,
            'joystick_debug',
            10
        )
        
        # Timer for command publishing and watchdog
        self.create_timer(0.02, self.publish_commands)  # 50Hz
        self.create_timer(1.0, self.watchdog_callback)  # 1Hz watchdog
        
        self.get_logger().info('Joystick controller node initialized')
    
    def joy_callback(self, msg):
        """
        Process incoming joystick messages
        
        Args:
            msg (Joy): Joystick message containing axes and buttons
        """
        try:
            self.last_joy_time = self.get_clock().now()
            
            # Process buttons
            self.process_buttons(msg.buttons)
            
            # Process axes
            self.process_axes(msg.axes)
            
            # Debug output if enabled
            if self.get_parameter('debug_enabled').value:
                debug_msg = String()
                debug_msg.data = f"Joy received: axes={msg.axes[:4]}, buttons={msg.buttons[:6]}"
                self.debug_publisher.publish(debug_msg)
                
        except Exception as e:
            self.get_logger().error(f'Error processing joystick input: {str(e)}')
    
    def process_buttons(self, buttons):
        """
        Process joystick buttons and update control states
        
        Args:
            buttons (list): List of button states (0 or 1)
        """
        button_mode = self.get_parameter('button_mode').value
        
        # Emergency stop (always in hold mode)
        if len(buttons) > self.button_indices['emergency_stop']:
            self.emergency_stop = buttons[self.button_indices['emergency_stop']] == 1
        
        # Gait mode selection
        if len(buttons) > self.button_indices['gait_mode']:
            if button_mode == 'toggle':
                if (buttons[self.button_indices['gait_mode']] == 1 and
                    not self.button_states.get('gait_mode', False)):
                    # Cycle through gait modes
                    if self.current_gait == 'normal':
                        self.current_gait = 'high'
                    elif self.current_gait == 'high':
                        self.current_gait = 'low'
                    else:
                        self.current_gait = 'normal'
            else:  # hold mode
                if buttons[self.button_indices['gait_mode']] == 1:
                    self.current_gait = 'high'
                else:
                    self.current_gait = 'normal'
        
        # Update button states for toggle mode
        for button_name, index in self.button_indices.items():
            if len(buttons) > index:
                self.button_states[button_name] = buttons[index] == 1
    
    def process_axes(self, axes):
        """
        Process joystick axes applying deadzone and exponential curve
        
        Args:
            axes (list): List of axis values (-1.0 to 1.0)
        """
        deadzone = self.get_parameter('joy_deadzone').value
        expo = self.get_parameter('joy_expo').value
        
        def apply_expo(value):
            """Apply exponential curve to axis value"""
            if abs(value) < deadzone:
                return 0.0
            sign = 1.0 if value > 0 else -1.0
            value = abs(value)
            return sign * (
                (1.0 - expo) * value +
                expo * value * value * value
            )
        
        # Process each axis
        for axis_name, index in self.axis_indices.items():
            if len(axes) > index:
                value = apply_expo(axes[index])
                setattr(self, f'{axis_name}_value', value)
    
    def publish_commands(self):
        """Convert joystick state to RC commands and publish"""
        try:
            msg = RCCommand()
            
            # Convert joystick values to SBUS range
            def joy_to_sbus(value):
                """Convert joystick value (-1 to 1) to SBUS range"""
                return int(
                    TX16SConfig.LIMITS['SBUS']['CENTER'] +
                    value * (TX16SConfig.LIMITS['SBUS']['MAX'] - 
                            TX16SConfig.LIMITS['SBUS']['CENTER'])
                )
            
            # Map stick values
            msg.right_stick_x = joy_to_sbus(getattr(self, 'right_x_value', 0.0))
            msg.right_stick_y = joy_to_sbus(-getattr(self, 'right_y_value', 0.0))  # Invert Y
            msg.left_stick_x = joy_to_sbus(getattr(self, 'left_x_value', 0.0))
            msg.left_stick_y = joy_to_sbus(-getattr(self, 'left_y_value', 0.0))    # Invert Y
            
            # Map buttons to switch positions
            msg.switches = self.create_switch_values()
            
            # Speed control using triggers
            l2_value = getattr(self, 'l2_value', -1.0)  # -1.0 when not pressed
            r2_value = getattr(self, 'r2_value', -1.0)
            msg.speed_knob = (r2_value + 1.0) / 2.0  # Convert -1:1 to 0:1
            
            # Emergency stop
            msg.emergency_stop = self.emergency_stop
            
            self.rc_publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing commands: {str(e)}')
    
    def create_switch_values(self):
        """
        Create switch values array matching TX16S switch configuration
        
        Returns:
            list: Switch values in SBUS range
        """
        switches = []
        
        # SA: Gait mode
        if self.current_gait == 'high':
            switches.append(TX16SConfig.LIMITS['SBUS']['MIN'])
        elif self.current_gait == 'normal':
            switches.append(TX16SConfig.LIMITS['SBUS']['CENTER'])
        else:
            switches.append(TX16SConfig.LIMITS['SBUS']['MAX'])
        
        # SB: Speed mode (using R1/RB)
        switches.append(
            TX16SConfig.LIMITS['SBUS']['MAX'] if self.button_states.get('speed_mode', False)
            else TX16SConfig.LIMITS['SBUS']['MIN']
        )
        
        # SC: Special moves (using Triangle/Y)
        switches.append(
            TX16SConfig.LIMITS['SBUS']['MAX'] if self.button_states.get('special_move', False)
            else TX16SConfig.LIMITS['SBUS']['MIN']
        )
        
        # SD: Stance mode (using Square/X)
        switches.append(
            TX16SConfig.LIMITS['SBUS']['MAX'] if self.button_states.get('stance_mode', False)
            else TX16SConfig.LIMITS['SBUS']['MIN']
        )
        
        # SE: Emergency stop
        switches.append(
            TX16SConfig.LIMITS['SBUS']['MAX'] if self.emergency_stop
            else TX16SConfig.LIMITS['SBUS']['MIN']
        )
        
        # SF: Demo mode (using Circle/B)
        switches.append(
            TX16SConfig.LIMITS['SBUS']['MAX'] if self.button_states.get('demo_mode', False)
            else TX16SConfig.LIMITS['SBUS']['MIN']
        )
        
        return switches
    
    def watchdog_callback(self):
        """Monitor joystick connection status"""
        time_since_last = (self.get_clock().now() - self.last_joy_time).nanoseconds / 1e9
        
        if time_since_last > 0.5:  # No joystick data for 500ms
            self.get_logger().warning('No joystick input received')
            self.emergency_stop = True  # Auto-enable emergency stop
            
            # Publish debug message
            if self.get_parameter('debug_enabled').value:
                debug_msg = String()
                debug_msg.data = "Joystick timeout - enabling emergency stop"
                self.debug_publisher.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    
    node = JoystickControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 
