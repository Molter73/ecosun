import os
from threading import Thread
import sys
import select

if os.name == 'nt':
    import msvcrt
    import time
else:
    import tty
    import termios

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

running = True
msg = """
Control manual de EcoSun.

Utiliza A y D para rotar el panel solar.
"""


class ManualMover(Node):
    def __init__(self):
        super().__init__('manual_mover')
        self.publisher = self.create_publisher(Twist, 'topic', 10)
        self.listener = Thread(target=keyboardListener, args=[self])
        self.listener.start()

    def rotate(self, x):
        twist = newTwist()
        twist.angular.x = x
        self.publisher.publish(twist)

    def rotate_left(self):
        self.rotate(1.0)

    def rotate_right(self):
        self.rotate(-1.0)


def newTwist():
    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0
    return twist


def getKey(settings):
    if os.name == 'nt':
        timeout = 0.1
        startTime = time.time()
        while (1):
            if msvcrt.kbhit():
                if sys.version_info[0] >= 3:
                    return msvcrt.getch().decode()
                else:
                    return msvcrt.getch()
            elif time.time() - startTime > timeout:
                return ''

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def keyboardListener(node):
    global running
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    print('Running listener...')
    while running:
        key = getKey(settings)
        twist = newTwist()
        if key == 'a':
            twist.angular.x = 1.0
            print('Rotando <-')
        elif key == 'd':
            twist.angular.x = -1.0
            print('Rotando ->')
        else:
            if key == '\x03':
                running = False
                node.destroy_publisher(node.publisher)
            continue

        node.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    print(msg)
    manual_mover = ManualMover()
    rclpy.spin(manual_mover)

    manual_mover.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
