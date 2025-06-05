#!/usr/bin/env python3

import os
import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import DynamicInterfaceGroupValues
from control_msgs.msg import InterfaceValue

MAX_LIN_VEL = 1.0
MAX_ANG_VEL = 1.0
LIN_STEP = 0.01
ANG_STEP = 0.1

INSTRUCTIONS = """
< Robocops teleop controller >
-----------------------------
Movement:
   z    
q  s  d
   x    

z/s : Increase/decrease linear velocity
q/d : Increase/decrease angular velocity
a : Stop

GPIO toggles:
w : Toggle capture routine
x : Toggle unload routine
c : Toggle button routine
v : Toggle slope_up routine
b : Toggle slope_down routine

Emergency stop: e

CTRL-C to quit
"""

KEY_BINDINGS = {
    'z': (LIN_STEP, 0),
    's': (-LIN_STEP, 0),
    'q': (0, ANG_STEP),
    'd': (0, -ANG_STEP),
    ' ': (0, 0),
    'a': (0, 0),
}

gpio_order = ["capture", "unload", "button", "slope_up", "slope_down" "emergency"]

gpio_interfaces = {
    "capture": "active",
    "unload": "active",
    "button": "active",
    "slope_up": "active",
    "slope_down": "active",
    "emergency": "active"
}

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def clamp(value, min_val, max_val):
    return max(min_val, min(value, max_val))


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')

        self.publisher = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.gpio_pub = self.create_publisher(DynamicInterfaceGroupValues, '/gpio_controller/commands', 10)

        self.linear = 0.0
        self.angular = 0.0

        # Track state for toggling
        self.gpio_states = {name: False for name in gpio_order}

        print(INSTRUCTIONS)

    def update_velocity(self, key):
        if key in KEY_BINDINGS:
            lin_delta, ang_delta = KEY_BINDINGS[key]
            if key in [' ', 'a']:
                self.linear = 0.0
                self.angular = 0.0
            else:
                self.linear = clamp(self.linear + lin_delta, -MAX_LIN_VEL, MAX_LIN_VEL)
                self.angular = clamp(self.angular + ang_delta, -MAX_ANG_VEL, MAX_ANG_VEL)
            print(f"Linear: {self.linear:.2f} | Angular: {self.angular:.2f}")

    def publish_velocity(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = self.linear
        msg.twist.angular.z = self.angular
        self.publisher.publish(msg)

    def toggle_gpio(self, init_name):
        self.gpio_states[init_name] = not self.gpio_states[init_name]

        msg = DynamicInterfaceGroupValues()
        msg.interface_groups = gpio_order

        for name, interface_name in gpio_interfaces.items():
            group_msg = InterfaceValue()
            group_msg.interface_names = [interface_name]
            group_msg.values = [1.0 if self.gpio_states[name] else 0.0]
            msg.interface_values.append(group_msg)

        self.gpio_pub.publish(msg)
        print(f"{init_name.capitalize()} toggled to {'ON' if self.gpio_states[init_name] else 'OFF'}")


def main():
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = TeleopNode()

    try:
        while True:
            key = get_key()
            if key == '\x03':  # Ctrl-C
                break
            if key == 'w':
                node.toggle_gpio("capture")
            elif key == 'x':
                node.toggle_gpio("unload")
            elif key == 'c':
                node.toggle_gpio("button")
            elif key == 'v':
                node.toggle_gpio("slope_up")
            elif key == 'b':
                node.toggle_gpio("slope_down")

            node.update_velocity(key)
            node.publish_velocity()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.linear = 0.0
        node.angular = 0.0
        node.publish_velocity()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
