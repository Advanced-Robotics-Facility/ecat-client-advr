#!/usr/bin/env python3

import os
import sys
import yaml

def generate_valve_config(valve_count, control_mode, valve_type):
    default_valve = {
        "valve_type": valve_type,
        "control_mode": f"0x{control_mode.upper()}" if not control_mode.lower().startswith("0x") else control_mode.upper(),
        "gains": [0.0, 0.0, 0.0, 0.0, 0.0],
    }

    valves = {}
    for i in range(1, valve_count + 1):
        valves[f"valve_{i}"] = default_valve

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


if __name__ == "__main__":
    allowed_valve_types = ["ADVRF"]  # Add more if needed
    allowed_control_modes = ["3B", "D4", "DD", "0x00"]  # Customize as needed

    valve_count = get_valid_valve_count()

    # Validate control mode
    while True:
        control_mode = input(f"Enter control mode {allowed_control_modes}: ").upper()
        norm_mode = control_mode.replace("0X", "")
        if control_mode in allowed_control_modes or norm_mode in [m.replace("0X", "") for m in allowed_control_modes]:
            if norm_mode == "00":
                control_mode = "0x00"
            else:
                control_mode = f"0x{norm_mode}"
            break
        else:
            print(f"Invalid control mode. Please choose from {allowed_control_modes}.")

    # Validate valve type
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


