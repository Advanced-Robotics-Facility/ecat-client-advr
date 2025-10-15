#!/usr/bin/env python3

import os
import sys
import yaml

# ---------- Custom inline formatting ----------

class InlineList(list): pass
def represent_inline_list(dumper, data):
    return dumper.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)
yaml.add_representer(InlineList, represent_inline_list)

class InlineDict(dict): pass
def represent_inline_dict(dumper, data):
    return dumper.represent_mapping('tag:yaml.org,2002:map', data, flow_style=True)
yaml.add_representer(InlineDict, represent_inline_dict)

# ---------- Config generator ----------

def sanitize_mode(mode):
    return mode.lower().replace("0x", "")

def get_positive_int(prompt, default=None):
    while True:
        try:
            val = input(prompt) if default is None else default
            val = int(val)
            if val >= 0:
                return val
            else:
                print("❌ Please enter a number ≥ 0.")
                default = None
        except ValueError:
            print("❌ Invalid input. Enter a valid integer.")
            default = None

def get_control_mode_from_arg_or_input(arg_mode, allowed_modes,device):
    # Normalize allowed modes without 0x for easier comparison
    allowed_norm = [m.replace("0X", "") for m in allowed_modes]

    # Validate arg_mode if provided
    if arg_mode:
        mode = arg_mode.upper()
        norm_mode = mode.replace("0X", "")
        if mode in allowed_modes or norm_mode in allowed_norm:
            if norm_mode == "00":
                return "0x00"
            else:
                return f"0x{norm_mode}"
        else:
            print(f"❌ Invalid control mode argument: {arg_mode}")
            # Fall through to interactive input

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



def generate_robot_yaml(num_motors, num_valves, num_pumps,
                        motor_control_mode, valve_control_mode, pump_control_mode):

    motor_ids = list(range(1, num_motors + 1))
    valve_start = num_motors + 1
    valve_ids = list(range(valve_start, valve_start + num_valves))

    pump_start = valve_start + num_valves
    pump_ids = list(range(pump_start, pump_start + num_pumps))

    data = {
        "trajectory": {
            "type": "sine",
            "frequency": 1,
            "repeat": 2,
        }
    }

    if num_motors > 0:
        data["motor"] = {
            "config_path": f"${{PWD}}/motor/robot_cfg_motor_{sanitize_mode(motor_control_mode)}.yaml",
            "id": InlineList(motor_ids),
            "set_point": InlineDict({
                "position": 1.0,
                "velocity": 2.0,
                "torque": 10.0,
                "current": 2,
            }),
            "homing": [InlineList([0.0] * num_motors)],
            "trajectory": [InlineList([0.0] * num_motors)],
        }

    if num_valves > 0:
        data["valve"] = {
            "config_path": f"${{PWD}}/valve/robot_cfg_valve_{sanitize_mode(valve_control_mode)}.yaml",
            "id": InlineList(valve_ids),
            "set_point": InlineDict({
                "current": 2.5,
                "position": 10.0,
                "force": 10.0,
            }),
        }

    if num_pumps > 0:
        data["pump"] = {
            "config_path": f"${{PWD}}/pump/robot_cfg_pump_{sanitize_mode(pump_control_mode)}.yaml",
            "id": InlineList(pump_ids),
            "set_point": InlineDict({
                "pressure": 128.0,
                "velocity": 10.0,
                "pwm": 30.0,
            }),
        }

    # Dump YAML to string first
    yaml_str = yaml.dump(data, sort_keys=False)

    # Add a blank line after every top-level block key by adding \n before each top-level key except the first
    lines = yaml_str.splitlines()
    new_lines = []
    for i, line in enumerate(lines):
        if i != 0 and line and not line.startswith(" "):
            # Insert blank line before new top-level key
            new_lines.append("")
        new_lines.append(line)
    yaml_str = "\n".join(new_lines)
    
    # Get the directory where the current script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))
    filename = "robot_contro_gen.yaml"
    filepath = os.path.join(script_dir, filename)

    # Write to file
    output_file = "robot_full_config.yaml"
    with open(filepath, "w") as f:
        f.write(yaml_str)

    print(f"✅ Saved config to {output_file}")

# ---------- Main CLI logic ----------

if __name__ == "__main__":
    args = sys.argv[1:]

    try:
        num_motors = get_positive_int("Enter number of motors: ", args[0] if len(args) > 0 else None)
        num_valves = get_positive_int("Enter number of valves: ", args[1] if len(args) > 1 else None)
        num_pumps = get_positive_int("Enter number of pumps: ", args[2] if len(args) > 2 else None)
    except IndexError:
        print("❌ Error parsing arguments.")
        sys.exit(1)
    
    # Example reading args safely, default to None if not provided
    arg_motor_mode = args[3] if len(args) > 3 else None
    arg_valve_mode = args[4] if len(args) > 4 else None
    arg_pump_mode = args[5] if len(args) > 5 else None
    
    allowed_control_modes = ["3B", "71", "D4", "CC", "DD", "0x00"]
    motor_mode = get_control_mode_from_arg_or_input(arg_motor_mode, allowed_control_modes,"Motor")
    
    allowed_control_modes = ["3B", "D4", "DD", "0x00"]
    valve_mode = get_control_mode_from_arg_or_input(arg_valve_mode, allowed_control_modes,"Valve")
    
    allowed_control_modes = ["71", "D4", "39", "0x00"]
    pump_mode = get_control_mode_from_arg_or_input(arg_pump_mode, allowed_control_modes,"Pump")


    generate_robot_yaml(num_motors, num_valves, num_pumps,
                        motor_mode, valve_mode, pump_mode)

