#!/usr/bin/env python3

class TX16SConfig:
    """
    Configuration and utilities for working with Radiomaster TX16S
    """
    
    # Base channel values
    CHANNEL_CONFIG = {
        # Main stick channels (Mode 2)
        'right_x': 0,    # Aileron - right/left movement
        'right_y': 1,    # Elevator - forward/backward movement
        'left_y': 2,     # Throttle - stance height
        'left_x': 3,     # Rudder - rotation
        
        # Switches
        'sa': 4,         # 3-position: walking mode
        'sb': 5,         # 3-position: speed presets
        'sc': 6,         # 3-position: special movements
        'sd': 7,         # 3-position: stance width
        'se': 8,         # 2-position: emergency stop
        'sf': 9,         # 2-position: demo mode
        
        # Potentiometers
        's1': 10,        # Left knob: fine speed adjustment
        's2': 11,        # Right knob: fine height adjustment
    }

    # Value limits
    LIMITS = {
        'SBUS': {
            'MIN': 172,
            'MAX': 1811,
            'CENTER': 992,
            'DEADZONE': 50
        },
        'PPM': {
            'MIN': 1000,
            'MAX': 2000,
            'CENTER': 1500,
            'DEADZONE': 30
        }
    }

    # Walking modes (SA switch)
    GAIT_MODES = {
        'up': 'high_gait',      # High stance gait
        'middle': 'normal_gait', # Normal stance gait
        'down': 'low_gait'      # Low stance gait
    }

    # Speed presets (SB switch)
    SPEED_PRESETS = {
        'up': 1.0,      # Maximum speed
        'middle': 0.7,  # Medium speed
        'down': 0.4     # Low speed
    }

    # Special movements (SC switch)
    SPECIAL_MOVES = {
        'up': 'wave',        # Wave leg
        'middle': 'none',    # Normal movement
        'down': 'dance'      # Dance mode
    }

    # Stance widths (SD switch)
    STANCE_WIDTHS = {
        'up': 'wide',       # Wide stance
        'middle': 'normal', # Normal stance
        'down': 'narrow'    # Narrow stance
    }

    @staticmethod
    def normalize_channel(value, protocol='SBUS'):
        """
        Normalize channel value to range -1.0 to 1.0
        
        Args:
            value: Channel value
            protocol: Protocol type ('SBUS' or 'PPM')
            
        Returns:
            float: Normalized value from -1.0 to 1.0
        """
        limits = TX16SConfig.LIMITS[protocol]
        
        # Check center deadzone
        if abs(value - limits['CENTER']) < limits['DEADZONE']:
            return 0.0
            
        # Normalize positive values
        if value > limits['CENTER']:
            return (value - limits['CENTER']) / (limits['MAX'] - limits['CENTER'])
        # Normalize negative values
        else:
            return (value - limits['CENTER']) / (limits['CENTER'] - limits['MIN'])

    @staticmethod
    def get_switch_position(value, is_3pos=True, protocol='SBUS'):
        """
        Determine switch position
        
        Args:
            value: Channel value
            is_3pos: True for 3-position, False for 2-position switch
            protocol: Protocol type ('SBUS' or 'PPM')
            
        Returns:
            str: 'up', 'middle', or 'down' for 3-pos; 'up' or 'down' for 2-pos
        """
        limits = TX16SConfig.LIMITS[protocol]
        
        if is_3pos:
            first_third = limits['MIN'] + (limits['MAX'] - limits['MIN']) / 3
            second_third = limits['MIN'] + 2 * (limits['MAX'] - limits['MIN']) / 3
            
            if value < first_third:
                return 'up'
            elif value < second_third:
                return 'middle'
            else:
                return 'down'
        else:
            return 'up' if value < (limits['MIN'] + limits['MAX']) / 2 else 'down'

    @staticmethod
    def map_range(value, in_min, in_max, out_min, out_max):
        """
        Map value from one range to another
        
        Args:
            value: Source value
            in_min: Input range minimum
            in_max: Input range maximum
            out_min: Output range minimum
            out_max: Output range maximum
            
        Returns:
            float: Mapped value
        """
        return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    @staticmethod
    def process_stick_command(value, protocol='SBUS', exponential=0.3):
        """
        Process stick command with exponential curve
        
        Args:
            value: Channel value
            protocol: Protocol type ('SBUS' or 'PPM')
            exponential: Exponential coefficient (0.0 - 1.0)
            
        Returns:
            float: Processed value from -1.0 to 1.0
        """
        # Normalize value
        normalized = TX16SConfig.normalize_channel(value, protocol)
        
        # Apply exponential curve
        if exponential > 0:
            sign = 1 if normalized > 0 else -1
            expo_value = sign * (
                (1 - exponential) * abs(normalized) +
                exponential * abs(normalized) ** 3
            )
            return expo_value
        return normalized

    @staticmethod
    def sbus_to_ppm(sbus_value):
        """
        Convert SBUS format value to PPM
        
        Args:
            sbus_value: SBUS value (172-1811)
            
        Returns:
            int: PPM value (1000-2000)
        """
        return int(TX16SConfig.map_range(
            sbus_value,
            TX16SConfig.LIMITS['SBUS']['MIN'],
            TX16SConfig.LIMITS['SBUS']['MAX'],
            TX16SConfig.LIMITS['PPM']['MIN'],
            TX16SConfig.LIMITS['PPM']['MAX']
        ))

    @staticmethod
    def ppm_to_sbus(ppm_value):
        """
        Convert PPM format value to SBUS
        
        Args:
            ppm_value: PPM value (1000-2000)
            
        Returns:
            int: SBUS value (172-1811)
        """
        return int(TX16SConfig.map_range(
            ppm_value,
            TX16SConfig.LIMITS['PPM']['MIN'],
            TX16SConfig.LIMITS['PPM']['MAX'],
            TX16SConfig.LIMITS['SBUS']['MIN'],
            TX16SConfig.LIMITS['SBUS']['MAX']
        ))

# Usage example
if __name__ == "__main__":
    # Test configuration
    
    # Test SBUS normalization
    print("SBUS value 992 (center) ->", TX16SConfig.normalize_channel(992, 'SBUS'))
    print("SBUS value 1811 (max) ->", TX16SConfig.normalize_channel(1811, 'SBUS'))
    print("SBUS value 172 (min) ->", TX16SConfig.normalize_channel(172, 'SBUS'))
    
    # Test switch position detection
    print("\nSwitch positions (SBUS):")
    print("Value 172 ->", TX16SConfig.get_switch_position(172, True, 'SBUS'))
    print("Value 992 ->", TX16SConfig.get_switch_position(992, True, 'SBUS'))
    print("Value 1811 ->", TX16SConfig.get_switch_position(1811, True, 'SBUS'))
    
    # Test format conversion
    sbus_value = 992
    ppm_value = TX16SConfig.sbus_to_ppm(sbus_value)
    back_to_sbus = TX16SConfig.ppm_to_sbus(ppm_value)
    print(f"\nFormat conversion: SBUS {sbus_value} -> PPM {ppm_value} -> SBUS {back_to_sbus}")
    
    # Test stick processing with expo
    print("\nStick processing with expo:")
    for value in [172, 582, 992, 1402, 1811]:
        processed = TX16SConfig.process_stick_command(value, 'SBUS', 0.3)
        print(f"Raw {value} -> Processed {processed:.2f}")
