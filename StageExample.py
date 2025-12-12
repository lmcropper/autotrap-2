#!/usr/bin/env python3
"""
Dual KDC101 controller for Raspberry Pi
Controls two motors (Z825B) with auto-detection, correct unit scaling,
and robust feedback-based moves.
"""

import time
import glob
from pylablib.devices import Thorlabs

# ------------------------------------------------------------
# CONFIGURATION
# ------------------------------------------------------------
# Put your motor serial numbers here
TARGET_SERIAL_A = "27250001"  # Motor A Serial Number
TARGET_SERIAL_B = "27250002"  # Motor B Serial Number

# Motor scale (steps/mm) for the Z825B
MOTOR_SCALE = 34304

# Wait time in seconds between distinct moves
WAIT_TIME_BETWEEN_MOVES = 3.0


# ------------------------------------------------------------
# Auto-detection function
# ------------------------------------------------------------
def find_motors():
    """
    Scans all /dev/ttyUSB* ports and returns a dictionary
    mapping serial numbers to their port.
    """
    print("[INFO] Scanning for Kinesis motors...")
    
    potential_ports = glob.glob("/dev/ttyUSB*")
    motor_map = {}
    
    if not potential_ports:
        print("[WARN] No /dev/ttyUSB devices found.")
        return {}

    for port in potential_ports:
        m = None
        try:
            m = Thorlabs.KinesisMotor(port)
            m.open()
            sn = m.get_device_info()[0] 
            print(f"[INFO] Found motor {sn} on port {port}")
            motor_map[str(sn)] = port
        
        except Exception as e:
            print(f"[WARN] Could not check port {port}. (In use or not a motor?)")
        
        finally:
            if m and m.is_opened():
                m.close()

    return motor_map


# ------------------------------------------------------------
# Homing wrapper (with true blocking wait)
# ------------------------------------------------------------
def home_motor(motor, name):
    print(f"\n[INFO] Homing {name}...")
    try:
        motor.home()
        while motor.is_moving():
            time.sleep(0.1)
        print(f"[INFO] {name} homed successfully.")
        
    except Exception as e:
        raise RuntimeError(f"Homing failed for {name}: {e}")


# ------------------------------------------------------------
# Move wrapper (UPDATED with feedback logic)
# ------------------------------------------------------------
def move_motor(motor, name, target_pos):
    print(f"\n[INFO] Moving {name} to {target_pos} mm...")
    try:
        # 1. ADDED: Get current position (The Feedback)
        current_pos = motor.get_position()
        print(f"[INFO] {name} is currently at {current_pos:.3f} mm")

        # 2. ADDED: Do the math
        distance_to_move = target_pos - current_pos
        print(f"[INFO] {name} needs to move by {distance_to_move:.3f} mm")
        
        # 3. CHANGED: Use the relative move command
        motor.move_by(distance_to_move)

        # The robust wait loop is still the same
        while motor.is_moving():
            time.sleep(0.1)  # Poll the status every 100ms

        # Add a tiny delay for the position to update before we read it
        time.sleep(0.05) 
        new_pos = motor.get_position()
        print(f"[INFO] {name} reached new position {new_pos:.3f} mm")
        
    except Exception as e:
        raise RuntimeError(f"Move failed for {name}: {e}")


# ------------------------------------------------------------
# MAIN PROGRAM (With robust init and cleanup)
# ------------------------------------------------------------
def main():
    print("\n========== KDC101 Auto-Detect Controller Script ==========")
    motorA = None
    motorB = None
    
    try:
        motor_ports = find_motors()
        
        if TARGET_SERIAL_A not in motor_ports:
            raise RuntimeError(f"Could not find motor {TARGET_SERIAL_A}!")
        if TARGET_SERIAL_B not in motor_ports:
            raise RuntimeError(f"Could not find motor {TARGET_SERIAL_B}!")
            
        port_A = motor_ports[TARGET_SERIAL_A]
        port_B = motor_ports[TARGET_SERIAL_B]
        
        print(f"\n[INFO] Opening motor {TARGET_SERIAL_A} on port {port_A}...")
        motorA = Thorlabs.KinesisMotor(port_A, scale=MOTOR_SCALE) 
        motorA.open()
        print(f"[INFO] Connected to: {motorA.get_device_info()}")

        print(f"\n[INFO] Opening motor {TARGET_SERIAL_B} on port {port_B}...")
        motorB = Thorlabs.KinesisMotor(port_B, scale=MOTOR_SCALE)
        motorB.open()
        print(f"[INFO] Connected to: {motorB.get_device_info()}")

        home_motor(motorA, "Motor A")
        home_motor(motorB, "Motor B")

        move_motor(motorA, "Motor A", 3.0)
        move_motor(motorB, "Motor B", 8.0)

        print(f"\n[INFO] Waiting {WAIT_TIME_BETWEEN_MOVES} seconds...")
        time.sleep(WAIT_TIME_BETWEEN_MOVES)

        move_motor(motorA, "Motor A", 1.0)
        move_motor(motorB, "Motor B", 1.0)

    except Exception as e:
        print(f"\n[FATAL ERROR] An error occurred: {e}")
        print("Check that:")
        print(" 1. Both devices are plugged in and powered on.")
        print(" 2. You have rebooted after adding user to 'dialout' group.")

    finally:
        print("\n[INFO] Cleanup: Closing devices...")
        if motorA and motorA.is_opened():
            motorA.close()
            print("[INFO] Motor A closed.")
        
        if motorB and motorB.is_opened():
            motorB.close()
            print("[INFO] Motor B closed.")
        
        print("\n✅ Script finished.\n")


# ------------------------------------------------------------
if __name__ == "__main__":
    main()
