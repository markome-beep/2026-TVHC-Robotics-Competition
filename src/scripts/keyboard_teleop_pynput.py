#!/usr/bin/env python3
"""Keyboard teleoperation for RC car using pynput for continuous key detection."""
import rclpy
import time
from geometry_msgs.msg import Twist
from pynput import keyboard

class KeyboardTeleop:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('keyboard_teleop')
        self.pub = self.node.create_publisher(Twist, '/ackermann_steering_controller/reference_unstamped', 10)

        # Control parameters
        self.speed = 0.0
        self.steer = 0.0
        self.max_speed = 5.0
        self.max_steer = 2.5

        # Key states - tracks which keys are currently held down
        self.keys_pressed = set()

        # Create listener
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release
        )
        self.listener.start()

    def on_press(self, key):
        """Called when a key is pressed."""
        try:
            # Try to get the character
            self.keys_pressed.add(key.char)
        except AttributeError:
            # Special keys (arrows, etc.)
            self.keys_pressed.add(key)

    def on_release(self, key):
        """Called when a key is released."""
        try:
            self.keys_pressed.discard(key.char)
        except AttributeError:
            self.keys_pressed.discard(key)
        
        # Check for quit
        try:
            if key.char == 'q':
                return False  # Stop listener
        except AttributeError:
            pass

    def update_controls(self):
        """Update speed and steering based on currently held keys."""
        self.speed = 0.0
        self.steer = 0.0
        
        # Check speed keys
        if 'w' in self.keys_pressed or 'W' in self.keys_pressed or keyboard.Key.up in self.keys_pressed:
            self.speed = self.max_speed
        elif 's' in self.keys_pressed or 'S' in self.keys_pressed or keyboard.Key.down in self.keys_pressed:
            self.speed = -self.max_speed
        
        # Check steering keys
        if 'a' in self.keys_pressed or 'A' in self.keys_pressed or keyboard.Key.left in self.keys_pressed:
            self.steer = self.max_steer if self.speed >= 0 else -self.max_steer
        elif 'd' in self.keys_pressed or 'D' in self.keys_pressed or keyboard.Key.right in self.keys_pressed:
            self.steer = -self.max_steer if self.speed >= 0 else self.max_steer
        
        # Emergency stop
        if ' ' in self.keys_pressed:
            self.speed = 0.0
            self.steer = 0.0

    def publish_command(self):
        """Publish the current command."""
        msg = Twist()
        msg.linear.x = self.speed
        msg.angular.z = self.steer
        self.pub.publish(msg)

    def print_status(self):
        """Print current status."""
        print(f"\rSpeed: {self.speed:5.2f} m/s | Steer: {self.steer:5.2f} rad", end='', flush=True)

    def run(self):
        """Main teleop loop."""
        print("╔════════════════════════════════════════╗")
        print("║   RC Car Keyboard Teleoperation (Py)   ║")
        print("╠════════════════════════════════════════╣")
        print("║  W / ↑     : forward                   ║")
        print("║  S / ↓     : reverse                   ║")
        print("║  A / ←     : turn right                ║")
        print("║  D / →     : turn left                 ║")
        print("║  SPACE     : emergency stop            ║")
        print("║  Q         : quit                      ║")
        print("╚════════════════════════════════════════╝")
        print("Speed:  0.00 m/s | Steer:  0.00 rad")

        try:
            while True:
                self.update_controls()
                self.publish_command()
                self.print_status()
                
                # Check if user pressed 'q'
                if 'q' in self.keys_pressed:
                    break
                
                time.sleep(0.01)  # 100Hz loop

        except KeyboardInterrupt:
            pass
        finally:
            self.listener.stop()
            # Publish zero command on exit
            self.speed = 0.0
            self.steer = 0.0
            self.publish_command()
            rclpy.shutdown()
            print("\nTeleop stopped.")

def main():
    try:
        teleop = KeyboardTeleop()
        teleop.run()
    except ImportError:
        print("\nError: pynput not installed!")
        print("Install it with: pip3 install pynput")
        return 1

if __name__ == '__main__':
    main()
