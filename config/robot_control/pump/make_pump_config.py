#!/usr/bin/env python3

import os
import sys
import yaml

def generate_pump_config(pump_count, control_mode, pump_type):
    default_pump = {
        "pump_type": pump_type,
        "control_mode": f"0x{control_mode.upper()}" if not control_mode.lower().startswith("0x") else control_mode.upper(),
        "gains": [0.0, 0.0, 0.0, 0.0, 0.0],
    }

    pumps = {}
    for i in range(1, pump_count + 1):
        pumps[f"pump_{i}"] = default_pump

    return pumps


def get_valid_pump_count():
    # Try command line argument first
    if len(sys.argv) > 1:
        try:
            count = int(sys.argv[1])
            if count <= 0:
                raise ValueError("Pump count must be greater than zero.")
            return count
        except ValueError as e:
            print(f"Invalid pump count argument: {e}")
            sys.exit(1)

    # If no valid CLI arg, prompt interactively
    while True:
        try:
            count = int(input("Enter number of pumps (>0): "))
            if count > 0:
                return count
            else:
                print("Pump count must be greater than zero. Please try again.")
        except ValueError:
            print("Invalid input. Please enter a positive integer.")


if __name__ == "__main__":
    allowed_pump_types = ["ADVRF"]
    allowed_control_modes = ["71", "D4", "39", "0x00"]

    pump_count = get_valid_pump_count()

    # Validate control mode
    while True:
        control_mode = input(f"Enter control mode {allowed_control_modes}: ").upper()
        norm_mode = control_mode.replace("0X", "")
        if control_mode in allowed_control_modes or norm_mode in [m.replace("0X","") for m in allowed_control_modes]:
            if norm_mode == "00":
                control_mode = "0x00"
            else:
                control_mode = f"0x{norm_mode}"
            break
        else:
            print(f"Invalid control mode. Please choose from {allowed_control_modes}.")

    # Validate pump type
    while True:
        pump_type = input(f"Enter pump type {allowed_pump_types}: ")
        if pump_type in allowed_pump_types:
            break
        else:
            print(f"Invalid pump type. Please choose from {allowed_pump_types}.")

    pumps = generate_pump_config(pump_count, control_mode, pump_type)

    # Get the directory where the current script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    sanitized_mode = control_mode.lower().replace("0x", "")
    filename = f"robot_cfg_pump_{sanitized_mode}.yaml"
    filepath = os.path.join(script_dir, filename)

    with open(filepath, "w") as f:
        yaml.dump(pumps, f, sort_keys=False)

    print(f"✅ Generated {filename} with {pump_count} pumps, control mode {control_mode}, pump type {pump_type}")

