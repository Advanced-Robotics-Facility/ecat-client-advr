#!/usr/bin/env python3

import os
import sys
import yaml

class InlineList(list): pass
def represent_inline_list(dumper, data):
    return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)
yaml.add_representer(InlineList, represent_inline_list)

def generate_valve_config(valve_count, control_mode, valve_type):
    # Normalize control_mode: if "idle" → "0x00"
    if control_mode == "idle":
        control_mode = "0x00"
    else:
        # Ensure control_mode string starts with "0x" and is uppercase
        control_mode = control_mode.upper()
        if not control_mode.startswith("0X"):
            control_mode = f"0x{control_mode}"
    
    default_valve_config = {
        "valve_type": valve_type,
        "brake_present": False,
        "control_mode": f"0x{control_mode.upper()}" if not control_mode.lower().startswith("0x") else control_mode.upper(),
    }
    
    # Add gains only if control_mode != "0x00"
    if control_mode != "0x00":
        gains=[0.0, 0.0, 0.0, 0.0, 0.0]
        default_valve_config["gains"] = InlineList(gains)

    
    valves = {}
    for i in range(1, valve_count + 1):
        valves[f"valve_{i}"] = default_valve_config

    return valves
    
def get_valid_valve_count():
    if len(sys.argv) > 1:
        try:
            count = int(sys.argv[1])
            if count <= 0:
                raise ValueError("Valve count must be greater than zero.")
            return count
        except ValueError as e:
            print(f"Invalid valve count argument: {e}")
            sys.exit(1)

    while True:
        try:
            count = int(input("Enter number of valves (>0): "))
            if count > 0:
                return count
            else:
                print("Valve count must be greater than zero. Please try again.")
        except ValueError:
            print("Invalid input. Please enter a positive integer.")

def get_control_mode_from_arg_or_input(arg_mode, allowed_modes,device):
    # Normalize allowed modes without 0x for easier comparison
    allowed_norm = [m.replace("0X", "").lstrip("0") or "0" for m in allowed_modes]

    # Validate arg_mode if provided
    if arg_mode:
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
    allowed_valve_types = ["ADVRF"]  # Add more if needed
    allowed_control_modes = ["3B", "D4", "DD", "0"]

    valve_count = get_valid_valve_count()
    arg_mode = sys.argv[2] if len(sys.argv) > 2 else None
    control_mode = get_control_mode_from_arg_or_input(arg_mode, allowed_control_modes,"Valve")

            
    # Automatically select valve type if only one is available
    if len(allowed_valve_types) == 1:
        valve_type = allowed_valve_types[0]
        print(f"✅ Only one valve type available. Using: {valve_type}") 
    else:
        # Validate valve type from user input
        while True:
           valve_type = input(f"Enter valve type {allowed_valve_types}: ")
           if valve_type in allowed_valve_types:
               break
           else:
               print(f"Invalid valve type. Please choose from {allowed_valve_types}.")

    valves = generate_valve_config(valve_count, control_mode, valve_type)

    # Get the directory where the current script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))

    sanitized_mode = control_mode.lower().replace("0x", "")
    filename = f"robot_cfg_valve_{sanitized_mode}.yaml"
    filepath = os.path.join(script_dir, filename)

    with open(filepath, "w") as f:
        yaml.dump(valves, f, sort_keys=False)

    print(f"✅ Generated {filename} with {valve_count} valves, control mode {control_mode}, valve type {valve_type}")


