from signal import signal, SIGINT
from threading import Thread

from flask import Flask, jsonify

import rclpy
from rclpy.node import Node

from ecosun_msgs.msg import Mode, Position


class EcosunListener(Node):
    def __init__(self):
        super().__init__('api')
        self.mode_sub = self.create_subscription(
            Mode,
            'cur_mode',
            self.mode_callback,
            10,
        )
        self.position_sub = self.create_subscription(
            Position,
            'cur_position',
            self.position_callback,
            10,
        )
        self.mode_sub  # prevent unused variable warning
        self.mode = ''
        self.position = -1

    def mode_callback(self, msg):
        self.mode = msg.mode
        print(f'Modo de operación: {self.mode}')

    def position_callback(self, msg):
        self.position = msg.position
        print(f'Posición actual: {self.position}')


def ros_runner(node):
    print('Escuchando posición y modo')
    rclpy.spin(node)


def sigint_handler(signal, frame):
    rclpy.shutdown()
    if prev_sigint_handler is not None:
        prev_sigint_handler(signal, frame)


rclpy.init(args=None)
ecosun_listener = EcosunListener()
app = Flask(__name__)
Thread(target=ros_runner, args=[ecosun_listener]).start()
prev_sigint_handler = signal(SIGINT, sigint_handler)


@app.route('/')
def get_status():
    response = jsonify({
        'mode': ecosun_listener.mode,
        'position': ecosun_listener.position,
    })

    response.headers.add('Access-Control-Allow-Origin', '*')

    return response
