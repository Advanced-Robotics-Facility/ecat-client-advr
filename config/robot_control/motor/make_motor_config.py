#!/usr/bin/env python3

import os
import sys
import yaml

class InlineList(list): pass
def represent_inline_list(dumper, data):
    return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)
yaml.add_representer(InlineList, represent_inline_list)

def generate_motor_config(motor_count, control_mode, motor_type):
    # Normalize control_mode: if "idle" → "0x00"
    if control_mode == "idle":
        control_mode = "0x00"
    else:
        # Ensure control_mode string starts with "0x" and is uppercase
        control_mode = control_mode.upper()
        if not control_mode.startswith("0X"):
            control_mode = f"0x{control_mode}"
    
    default_motor_config = {
        "motor_type": motor_type,
        "brake_present": False,
        "control_mode": f"0x{control_mode.upper()}" if not control_mode.lower().startswith("0x") else control_mode.upper(),
    }
    
    # Add gains only if control_mode != "0x00"
    if control_mode != "0x00":
        gains=[0.0, 0.0, 0.0, 0.0, 0.0]
        default_motor_config["gains"] = InlineList(gains)

    
    motors = {}
    for i in range(1, motor_count + 1):
        motors[f"motor_{i}"] = default_motor_config

    return motors

def get_valid_motor_count():
    if len(sys.argv) > 1:
        try:
            count = int(sys.argv[1])
            if count <= 0:
                raise ValueError("Motor count must be greater than zero.")
            return count
        except ValueError as e:
            print(f"Invalid motor count argument: {e}")
            sys.exit(1)

    while True:
        try:
            count = int(input("Enter number of motors (>0): "))
            if count > 0:
                return count
            else:
                print("Motor count must be greater than zero. Please try again.")
        except ValueError:
            print("Invalid input. Please enter a positive integer.")

def get_control_mode_from_arg_or_input(arg_mode, allowed_modes,device):
    # Normalize allowed modes without 0x for easier comparison
    allowed_norm = [m.replace("0X", "").lstrip("0") or "0" for m in allowed_modes]

    # Validate arg_mode if provided
    if arg_mode:
        if isinstance(arg_mode, list):
    	    arg_mode = arg_mode[0]
        mode = arg_mode.upper()
        norm_mode = mode.replace("0X", "").lstrip("0") or "0"
        if mode in allowed_modes or norm_mode in allowed_norm:
            if norm_mode == "0":
                return "idle"
            else:
                return f"0x{norm_mode}"
        else:
            print(f"❌ Invalid control mode argument: {arg_mode}")
            # Fall through to interactive input

    # Interactive input loop
    while True:
        control_mode = input(f"Enter control mode {allowed_modes} for {device}: ").upper()
        norm_mode = control_mode.replace("0X", "").lstrip("0") or "0"
        if control_mode in allowed_modes or norm_mode in allowed_norm:
            if norm_mode == "0":
                return "idle"
            else:
                return f"0x{norm_mode}"
        else:
            print(f"Invalid control mode. Please choose from {allowed_modes}.")


if __name__ == "__main__":
    allowed_motor_types = ["ADVRF", "Synapticon", "Novanta","Amc"]
    allowed_control_modes = ["3B", "71", "D4", "CC", "DD", "0"]

    
    motor_count = get_valid_motor_count()
    arg_mode = sys.argv[2] if len(sys.argv) > 2 else None
    control_mode = get_control_mode_from_arg_or_input(arg_mode, allowed_control_modes,"Motor")

    # Validate motor type
    while True:
        motor_type = input(f"Enter motor type {allowed_motor_types}: ")
        if motor_type in allowed_motor_types:
            break
        else:
            print(f"Invalid motor type. Please choose from {allowed_motor_types}.")

    motors = generate_motor_config(motor_count, control_mode, motor_type)

    # Get the directory where the current script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))

    sanitized_mode = control_mode.lower().replace("0x", "")
    filename = f"robot_cfg_motor_{sanitized_mode}.yaml"
    filepath = os.path.join(script_dir, filename)

    with open(filepath, "w") as f:
        yaml.dump(motors, f, sort_keys=False)

    print(f"✅ Generated {filename} with {motor_count} motors, control mode {control_mode}, motor type {motor_type}")


