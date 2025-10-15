#!/usr/bin/env python3

import yaml
import subprocess

def get_control_mode_from_input(allowed_modes,device):
    # Normalize allowed modes without 0x for easier comparison
    allowed_norm = [m.replace("0X", "") for m in allowed_modes]

    # Interactive input loop
    while True:
        control_mode = input(f"Enter control mode {allowed_modes} for {device}: ").upper()
        norm_mode = control_mode.replace("0X", "")
        if control_mode in allowed_modes or norm_mode in allowed_norm:
            if norm_mode == "00":
                return "0x00"
            else:
                return f"0x{norm_mode}"
        else:
            print(f"Invalid control mode. Please choose from {allowed_modes}.")

def get_positive_int(prompt):
    while True:
        try:
            value = int(input(prompt))
            if value >= 0:
                return value
            else:
                print("Value must be zero or a positive integer. Try again.")
        except ValueError:
            print("Invalid input. Please enter a valid integer.")

# Prompt the user
num_motors = get_positive_int("Enter number of motors (>=0): ")
num_valves = get_positive_int("Enter number of valves (>=0): ")
num_pumps = get_positive_int("Enter number of  pumps (>=0): ")

joint_map = {}

# Add motors
for i in range(1, num_motors + 1):
    joint_map[i] = f"motor_{i}"

# Add valves
start_valve_id = num_motors + 1
for i in range(1, num_valves + 1):
    joint_map[start_valve_id + i - 1] = f"valve_{i}"

# Add pumps
start_pump_id = num_motors + num_valves + 1
for i in range(1, num_pumps + 1):
    joint_map[start_pump_id + i - 1] = f"pump_{i}"

# Write to YAML
with open("robot_id_map/robot_id_map_gen.yaml", "w") as file:
    yaml.dump({"joint_map": joint_map}, file, sort_keys=False)

print(f"✅ Created robot_id_map_gen.yaml with {num_motors} motors, {num_valves} valves, and {num_pumps} pumps.")

# Run motor cfg script
motor_ctrl_mode=[0x00]
if num_motors > 0:
    allowed_control_modes = ["3B", "71", "D4", "CC", "DD", "0x00"]
    motor_ctrl_mode=get_control_mode_from_input(allowed_control_modes,"Motor")
    subprocess.run([
        "python3",
        "robot_control/motor/make_motor_config.py",
        str(num_motors),
        str(motor_ctrl_mode)
    ])

# Run valve cfg script
valve_ctrl_mode=[0x00]
if num_valves > 0:
    allowed_control_modes = ["3B", "D4", "DD", "0x00"]
    valve_ctrl_mode=get_control_mode_from_input(allowed_control_modes,"Valve")
    subprocess.run([
        "python3",
        "robot_control/valve/make_valve_config.py",
        str(num_valves),
        str(valve_ctrl_mode)
    ])

# Run valve cfg script
pump_ctrl_mode=[0x00]
if num_pumps > 0:
    allowed_control_modes = ["71", "D4", "39", "0x00"]
    pump_ctrl_mode=get_control_mode_from_input(allowed_control_modes,"Pump")
    subprocess.run([
        "python3",
        "robot_control/pump/make_pump_config.py",
        str(num_pumps),
        str(pump_ctrl_mode)
    ])
    
subprocess.run([
    "python3", 
    "robot_control/make_robot_control_cfg.py",
    str(num_motors),str(num_valves),str(num_pumps),str(motor_ctrl_mode),str(valve_ctrl_mode),str(pump_ctrl_mode)
])

