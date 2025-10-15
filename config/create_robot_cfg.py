#!/usr/bin/env python3

import yaml
import subprocess

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
num_pumps = get_positive_int("Enter number of pumps (>=0): ")

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
if num_motors > 0:
    subprocess.run([
        "python3",
        "robot_control/motor/make_motor_config.py",
        str(num_motors)  # Convert the integer to string
    ])

# Run valve cfg script
if num_valves > 0:
    subprocess.run([
        "python3",
        "robot_control/valve/make_valve_config.py",
        str(num_valves)  # Convert the integer to string
    ])

# Run valve cfg script
if num_pumps > 0:
    subprocess.run([
        "python3",
        "robot_control/pump/make_pump_config.py",
        str(num_pumps)  # Convert the integer to string
    ])

