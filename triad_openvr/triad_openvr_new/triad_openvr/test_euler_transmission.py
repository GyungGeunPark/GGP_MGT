#!/usr/bin/env python
"""
Test script for verifying Euler angle transmission and coordinate system conversion.
This script helps test the VR tracking system after implementing Euler angle changes.
"""

import triad_openvr
import time
import sys
import math

def print_header():
    """Print test header information"""
    print("=" * 80)
    print("VR Tracker Euler Angle Transmission Test")
    print("=" * 80)
    print("This test helps verify the coordinate system transformation")
    print("between OpenVR and Unity after implementing Euler angle transmission.")
    print()
    print("Instructions:")
    print("1. Start this script")
    print("2. Start Unity with tcp_tracked_object attached to an object")
    print("3. Move the tracker along each axis and observe the output")
    print("4. Check if movements match expected directions")
    print("=" * 80)
    print()

def test_axis_movement(v, tracker_name="tracker_1"):
    """
    Test movement along each axis
    """
    print("\n" + "=" * 50)
    print("AXIS MOVEMENT TEST")
    print("=" * 50)
    print("Move the tracker along each axis and verify:")
    print("- X axis: Should move left/right in Unity")
    print("- Y axis: Should move up/down in Unity")
    print("- Z axis: Should move forward/backward in Unity")
    print()
    print("Press Ctrl+C to stop...")
    print()
    
    try:
        last_pos = None
        while True:
            # Get pose data
            pose_euler = v.devices[tracker_name].get_pose_euler()
            
            if pose_euler:
                x, y, z, pitch, yaw, roll = pose_euler
                
                # Detect significant movement
                if last_pos:
                    dx = x - last_pos[0]
                    dy = y - last_pos[1]
                    dz = z - last_pos[2]
                    
                    # Detect which axis has the most movement
                    if abs(dx) > 0.01 or abs(dy) > 0.01 or abs(dz) > 0.01:
                        dominant_axis = ""
                        if abs(dx) > abs(dy) and abs(dx) > abs(dz):
                            dominant_axis = f"X-axis movement: {dx:+.3f}m"
                        elif abs(dy) > abs(dx) and abs(dy) > abs(dz):
                            dominant_axis = f"Y-axis movement: {dy:+.3f}m"
                        else:
                            dominant_axis = f"Z-axis movement: {dz:+.3f}m (Note: Z inverted for Unity)"
                        
                        print(f"\rMovement detected - {dominant_axis}    ", end="")
                
                last_pos = [x, y, z]
                
            time.sleep(0.05)  # 20Hz update
            
    except KeyboardInterrupt:
        print("\nAxis movement test stopped.")

def test_rotation(v, tracker_name="tracker_1"):
    """
    Test rotation around each axis
    """
    print("\n" + "=" * 50)
    print("ROTATION TEST")
    print("=" * 50)
    print("Rotate the tracker around each axis and verify:")
    print("- Pitch (X-axis): Tilt forward/backward")
    print("- Yaw (Y-axis): Turn left/right")
    print("- Roll (Z-axis): Tilt left/right")
    print()
    print("Press Ctrl+C to stop...")
    print()
    
    try:
        last_rot = None
        while True:
            # Get pose data
            pose_euler = v.devices[tracker_name].get_pose_euler()
            
            if pose_euler:
                x, y, z, pitch, yaw, roll = pose_euler
                
                # Detect significant rotation
                if last_rot:
                    dpitch = pitch - last_rot[0]
                    dyaw = yaw - last_rot[1]
                    droll = roll - last_rot[2]
                    
                    # Handle angle wrapping
                    if dpitch > 180: dpitch -= 360
                    if dpitch < -180: dpitch += 360
                    if dyaw > 180: dyaw -= 360
                    if dyaw < -180: dyaw += 360
                    if droll > 180: droll -= 360
                    if droll < -180: droll += 360
                    
                    # Detect which axis has the most rotation
                    if abs(dpitch) > 1 or abs(dyaw) > 1 or abs(droll) > 1:
                        dominant_rot = ""
                        if abs(dpitch) > abs(dyaw) and abs(dpitch) > abs(droll):
                            dominant_rot = f"Pitch rotation: {dpitch:+.1f}°"
                        elif abs(dyaw) > abs(dpitch) and abs(dyaw) > abs(droll):
                            dominant_rot = f"Yaw rotation: {dyaw:+.1f}°"
                        else:
                            dominant_rot = f"Roll rotation: {droll:+.1f}°"
                        
                        print(f"\rRotation detected - {dominant_rot}    ", end="")
                
                last_rot = [pitch, yaw, roll]
                
            time.sleep(0.05)  # 20Hz update
            
    except KeyboardInterrupt:
        print("\nRotation test stopped.")

def continuous_display(v, tracker_name="tracker_1"):
    """
    Continuously display position and rotation data
    """
    print("\n" + "=" * 50)
    print("CONTINUOUS DATA DISPLAY")
    print("=" * 50)
    print("Showing real-time position and rotation data...")
    print("Press Ctrl+C to stop...")
    print()
    
    try:
        while True:
            # Get pose data
            pose_euler = v.devices[tracker_name].get_pose_euler()
            
            if pose_euler:
                x, y, z, pitch, yaw, roll = pose_euler
                
                # Display formatted data
                pos_str = f"Pos: X={x:+.3f}m Y={y:+.3f}m Z={z:+.3f}m"
                rot_str = f"Rot: Pitch={pitch:+.1f}° Yaw={yaw:+.1f}° Roll={roll:+.1f}°"
                
                print(f"\r{pos_str} | {rot_str}    ", end="")
                
            time.sleep(0.1)  # 10Hz update
            
    except KeyboardInterrupt:
        print("\nContinuous display stopped.")

def verify_30_degree_offset(v, tracker_name="tracker_1"):
    """
    Help verify if there's a 30-degree offset issue
    """
    print("\n" + "=" * 50)
    print("30-DEGREE OFFSET VERIFICATION")
    print("=" * 50)
    print("This test helps identify if there's a 30-degree rotation offset.")
    print()
    print("Instructions:")
    print("1. Point the tracker straight forward (along Unity's Z-axis)")
    print("2. Move the tracker forward/backward")
    print("3. Check if movement is along Z-axis or rotated")
    print()
    print("Current orientation:")
    
    try:
        # Get initial orientation
        pose_euler = v.devices[tracker_name].get_pose_euler()
        if pose_euler:
            x, y, z, pitch, yaw, roll = pose_euler
            print(f"Initial Yaw: {yaw:.1f}°")
            print(f"If there's a 30° offset, yaw should be around ±30° when pointing forward")
            print()
        
        print("Now move the tracker forward/backward...")
        print("Press Ctrl+C to stop...")
        print()
        
        initial_pos = [x, y, z]
        
        while True:
            pose_euler = v.devices[tracker_name].get_pose_euler()
            if pose_euler:
                x, y, z, pitch, yaw, roll = pose_euler
                
                # Calculate movement from initial position
                dx = x - initial_pos[0]
                dy = y - initial_pos[1]
                dz = z - initial_pos[2]
                
                # Calculate angle of movement in XZ plane
                if abs(dx) > 0.01 or abs(dz) > 0.01:
                    movement_angle = math.degrees(math.atan2(dx, -dz))  # Note: -dz because Z is inverted
                    
                    print(f"\rMovement: X={dx:+.3f}m Z={dz:+.3f}m | " +
                          f"Movement angle: {movement_angle:+.1f}° | " +
                          f"Tracker Yaw: {yaw:+.1f}°    ", end="")
            
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\n30-degree offset verification stopped.")

def main():
    # Initialize OpenVR
    print("Initializing OpenVR...")
    v = triad_openvr.triad_openvr()
    v.print_discovered_objects()
    
    # Check if tracker is available
    tracker_name = "tracker_1"
    if tracker_name not in v.devices:
        print(f"Error: {tracker_name} not found!")
        print("Available devices:", list(v.devices.keys()))
        return
    
    print_header()
    
    while True:
        print("\n" + "=" * 50)
        print("SELECT TEST MODE:")
        print("=" * 50)
        print("1. Test axis movement (X, Y, Z)")
        print("2. Test rotation (Pitch, Yaw, Roll)")
        print("3. Continuous data display")
        print("4. Verify 30-degree offset")
        print("5. Exit")
        print()
        
        try:
            choice = input("Enter your choice (1-5): ").strip()
            
            if choice == '1':
                test_axis_movement(v, tracker_name)
            elif choice == '2':
                test_rotation(v, tracker_name)
            elif choice == '3':
                continuous_display(v, tracker_name)
            elif choice == '4':
                verify_30_degree_offset(v, tracker_name)
            elif choice == '5':
                print("Exiting...")
                break
            else:
                print("Invalid choice. Please enter 1-5.")
                
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    main()