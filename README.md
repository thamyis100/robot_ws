# robot_ws

This package contains the low-level robot control nodes for the base.
It handles joystick input, velocity mixing, serial communication, and motor enable/disable commands.

## What is inside

The package builds three executables:

- `controller_node`: reads the joystick and publishes teleop commands.
- `twist_mux`: chooses between teleop and navigation velocity commands.
- `serial_node`: converts `cmd_vel` into motor commands and sends them to the motor controller over serial.

## How it starts

The startup flow is simple:

1. `controller_node` opens the joystick device, usually `/dev/input/js0`.
2. It publishes `cmd_vel_teleop` and `activate`.
3. `twist_mux` listens to `cmd_vel_teleop` and `cmd_vel_nav`, then publishes one final `cmd_vel` output.
4. `serial_node` listens to `cmd_vel` and `activate`.
5. `serial_node` opens the serial port, usually `/dev/ttyUSB0`, and sends the motor enable or stop commands.
6. When `cmd_vel` arrives, `serial_node` converts the robot motion into wheel commands and writes them to the controller.

## Inputs

- Joystick device input from `/dev/input/js0`.
- Velocity input on `cmd_vel_teleop` and `cmd_vel_nav`.
- Activation input on `activate`.
- Serial device path, baud rate, wheel size, and base geometry parameters.

## Outputs

- `cmd_vel_teleop` from the joystick node.
- `activate` from the joystick node.
- `cmd_vel` from the twist mux.
- Serial commands sent to the motor controller through the configured port.
- Optional status data from the serial port is published on `serial/in`.

## Parameters

Common defaults used by the nodes:

- Joystick device: `/dev/input/js0`
- Serial port: `/dev/ttyUSB0`
- Serial baud rate: `2000000`
- Teleop output topic: `/cmd_vel_teleop`
- Navigation input topic: `/cmd_vel_nav`
- Final drive topic: `/cmd_vel`

## Notes

- `controller_node` publishes `activate` when the Y button is pressed.
- `twist_mux` gives teleop higher priority than navigation.
- `serial_node` sends motor enable commands when `activate` becomes `true`, and stop/disable commands when it becomes `false`.
- This folder is for the robot base control only; other packages in the workspace have their own documentation.