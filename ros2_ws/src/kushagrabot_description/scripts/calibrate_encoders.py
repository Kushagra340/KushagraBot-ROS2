#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time

class EncoderCalibrator(Node):
    def __init__(self):
        super().__init__('encoder_calibrator')
        self.subscription = self.create_subscription(
            Int32,
            '/left_ticks',
            self.ticks_callback,
            10)
        self.current_ticks = None
        
    def ticks_callback(self, msg):
        self.current_ticks = msg.data

def main(args=None):
    rclpy.init(args=args)
    calibrator = EncoderCalibrator()
    
    print("\n========================================")
    print("      INTERACTIVE ENCODER CALIBRATION   ")
    print("========================================")
    print("We will calibrate this purely physically to eliminate motor inertia and wobble!")
    print("Place the robot on the floor and mark its starting position.")
    
    # Wait for first tick reading
    while calibrator.current_ticks is None:
        rclpy.spin_once(calibrator, timeout_sec=0.1)
        
    start_ticks = calibrator.current_ticks
    print(f"\nInitial Ticks Registered: {start_ticks}")
    
    input("\nNow, PUSH the robot forward BY HAND exactly 30 cm. Press Enter when done...")
    
    # Process any pending messages to get the latest ticks
    rclpy.spin_once(calibrator, timeout_sec=0.5)
    end_ticks = calibrator.current_ticks
    
    ticks_traveled = abs(end_ticks - start_ticks)
    print(f"Final Ticks Registered: {end_ticks}")
    print(f"Total Ticks Traveled: {ticks_traveled}")
    
    if ticks_traveled < 10:
        print("ERROR: Not enough ticks registered. Is the ESP32 publishing /left_ticks?")
    else:
        # Math:
        # Wheel circumference = 3.14159 * 3.4 cm = 10.68 cm
        # 30 cm travel = 30 / 10.68 = 2.808 revolutions
        # CPR = ticks_traveled / 2.808
        circumference = 3.14159 * 3.4
        revolutions = 30.0 / circumference
        new_cpr = ticks_traveled / revolutions
        
        print("\n========================================")
        print("          CALIBRATION COMPLETE          ")
        print("========================================")
        print(f"Physical distance: 30.0 cm")
        print(f"Wheel Revolutions: {revolutions:.2f}")
        print(f"\n==> YOUR NEW ENCODER_CPR = {int(new_cpr)} <==")
        print("\nNext Steps:")
        print("1. Open main.cpp and change ENCODER_CPR to this exact integer value.")
        print("2. Run ./build_firmware_fix.sh one last time to flash it.")
        
    calibrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
