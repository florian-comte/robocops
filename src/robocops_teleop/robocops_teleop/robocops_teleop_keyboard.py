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

z/x : Increase/decrease linear velocity
q/d : Increase/decrease angular velocity
s or space : Stop

GPIO toggles:
w : Toggle brushes
c : Toggle lift
e : Toggle unload

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

gpio_order = ["brushes", "unload", "lift"]  # Define GPIO command group names


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

    def toggle_gpio(self, name):
        self.gpio_states[name] = not self.gpio_states[name]

        msg = DynamicInterfaceGroupValues()
        msg.interface_groups = gpio_order

        for group in gpio_order:
            group_msg = InterfaceValue()
            group_msg.interface_names = [group]
            group_msg.values = [1.0 if self.gpio_states[group] else 0.0]
            msg.interface_values.append(group_msg)

        print(msg)

        self.gpio_pub.publish(msg)
        print(f"{name.capitalize()} toggled to {'ON' if self.gpio_states[name] else 'OFF'}")


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
                node.toggle_gpio("brushes")
            elif key == 'e':
                node.toggle_gpio("unload")
            elif key == 'c':
                node.toggle_gpio("lift")

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
