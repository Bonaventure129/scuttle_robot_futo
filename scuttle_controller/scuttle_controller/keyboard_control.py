#!/usr/bin/env python3
import sys
import termios
import tty
import select
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class UniversalTeleop(Node):
    def __init__(self):
        super().__init__('universal_teleop')
        self.publisher_ = self.create_publisher(Twist, '/key_vel', 10)
        
        self.speed = 0.4
        self.turn = 1.0
        self.persistence = 0.25 # Keep moving for 0.25s after key release

        self.last_up = 0
        self.last_down = 0
        self.last_left = 0
        self.last_right = 0

        print("--------------------------------------------------")
        print(" UNIVERSAL TELEOP RUNNING")
        print(" keys are INVISIBLE but they are working.")
        print(" Press 'q' to quit.")
        print("--------------------------------------------------")

    def run(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        
        try:
            tty.setraw(sys.stdin.fileno())
            
            while rclpy.ok():
                rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
                
                if rlist:
                    key_buffer = sys.stdin.read(1024)
                    now = time.time()
                    
                    # --- DEBUG: Print what we detect (Carriage Return \r returns to start of line) ---
                    # This lets you see if keys are being registered!
                    if len(key_buffer) > 0:
                        print(f"Key Detected\r", end='')

                    # --- CHECK ALL KEY CODE VARIATIONS ---
                    # Standard ANSI vs Application Mode (common on ThinkPads)
                    if '\x1b[A' in key_buffer or '\x1bOA' in key_buffer: 
                        self.last_up = now
                    if '\x1b[B' in key_buffer or '\x1bOB' in key_buffer: 
                        self.last_down = now
                    if '\x1b[C' in key_buffer or '\x1bOC' in key_buffer: 
                        self.last_right = now
                    if '\x1b[D' in key_buffer or '\x1bOD' in key_buffer: 
                        self.last_left = now
                    if 'q' in key_buffer: 
                        break

                # --- CALCULATE MOVEMENT ---
                twist = Twist()
                now = time.time()
                
                # Check timestamps (Diagonal logic)
                if now - self.last_up < self.persistence:
                    twist.linear.x = self.speed
                elif now - self.last_down < self.persistence:
                    twist.linear.x = -self.speed
                
                if now - self.last_left < self.persistence:
                    twist.angular.z = self.turn
                elif now - self.last_right < self.persistence:
                    twist.angular.z = -self.turn

                self.publisher_.publish(twist)
                time.sleep(0.05)

        except Exception as e:
            print(f"Error: {e}\r")
        finally:
            # Stop robot and clean up terminal
            self.publisher_.publish(Twist())
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            print("\nStopped.\n")

def main(args=None):
    rclpy.init(args=args)
    node = UniversalTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()