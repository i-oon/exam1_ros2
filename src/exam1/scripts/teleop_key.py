#!/usr/bin/python3

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput import keyboard


class TeleopKey(Node):
    def __init__(self):
        super().__init__('teleop_key')

        self.debounce_press_ms = 100
        self.debounce_release_ms = 100

        self.key_pub = self.create_publisher(String, '/keyboard', 10)
        self.control_timer = self.create_timer(0.1, self.timer_callback)

        self.print_instructions()

        self.current_key = None

        # Track per-key state for debouncing
        self._pressed = {}
        self._last_press_ms = {}
        self._last_release_ms = {}

        # keys we allow to publish
        self.allowed_keys = set(list("wasdpocrfvgb"))

        try:
            self.get_logger().info("Initializing keyboard listener ...")
            self.keyboard_listener = keyboard.Listener(
                on_press=self.on_key_press,
                on_release=self.on_key_release
            )
            self.keyboard_listener.start()
            self.get_logger().info("Keyboard listener started.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize keyboard listener: {str(e)}")
            raise

    def now_ms(self):
        return int(time.monotonic() * 1000)

    def on_key_press(self, key):
        try:
            k = key.char.lower()
        except AttributeError:
            if key == keyboard.Key.esc:
                self._debounced_release('__esc__')
                return
            return

        if k not in self.allowed_keys:
            return

        if self._pressed.get(k, False):
            return

        now = self.now_ms()
        last = self._last_press_ms.get(k, -10**9)
        if now - last < self.debounce_press_ms:
            return

        self._pressed[k] = True
        self._last_press_ms[k] = now

        self.current_key = k

    def on_key_release(self, key):
        try:
            k = key.char.lower()
        except AttributeError:
            if key == keyboard.Key.esc:
                self._debounced_release('__esc__')
            return

        if k not in self.allowed_keys:
            return

        self._debounced_release(k)

    def _debounced_release(self, k):
        now = self.now_ms()
        last_rel = self._last_release_ms.get(k, -10**9)
        if now - last_rel < self.debounce_release_ms:
            return

        self._last_release_ms[k] = now
        self._pressed[k] = False

        if self.current_key == k or k == '__esc__':
            self.current_key = None

    def timer_callback(self):
        msg = String()
        if self.current_key in self.allowed_keys:
            msg.data = self.current_key
        else:
            msg.data = "None"
        self.key_pub.publish(msg)

    def print_instructions(self):
        banner = """
    Control Your Turtle Teleop
    ----------------------------------------
    Moving around:
            w
        a   s   d

    Other keys:
    p : spawn pizza
    o : save
    c : clear
    f/v : increase/decrease linear speed
    g/b : increase/decrease angular speed

    CTRL-C  : quit

    """
        print(banner)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKey()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
