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
from ecosun_msgs.msg import Rotate, Mode

running = True
msg = """
Control manual de EcoSun.

Utiliza A y D para rotar el panel solar.
S para modo de salvaguarda.
W para volver a operación normal.
"""


class Controller(Node):
    def __init__(self):
        super().__init__('manual_mover')
        self.rotation_pub = self.create_publisher(Rotate, 'rotate_dir', 10)
        self.mode_pub = self.create_publisher(Mode, 'set_mode', 10)
        self.listener = Thread(target=keyboardListener, args=[self])
        self.listener.start()

    def rotate(self, x):
        r = Rotate()
        r.dir = x
        self.rotation_pub.publish(r)

    def rotate_left(self):
        self.rotate(1)

    def rotate_right(self):
        self.rotate(-1)

    def goto_safeguard(self):
        mode = Mode()
        mode.mode = 'safeguard'
        self.mode_pub.publish(mode)

    def normal_operation(self):
        mode = Mode()
        mode.mode = 'normal'
        self.mode_pub.publish(mode)


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
        if key == 'a':
            print('Rotando <-')
            node.rotate_left()
        elif key == 'd':
            print('Rotando ->')
            node.rotate_right()
        elif key == 's':
            print('Yendo a safeguard')
            node.goto_safeguard()
        elif key == 'w':
            print('Yendo a operación normal')
            node.normal_operation()
        else:
            if key == '\x03':
                running = False
            continue


def main(args=None):
    rclpy.init(args=args)

    print(msg)
    controller = Controller()
    rclpy.spin(controller)

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
