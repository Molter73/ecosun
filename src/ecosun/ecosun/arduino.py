from threading import Thread

import rclpy
from rclpy.node import Node

from ecosun_msgs.msg import Rotate, Mode

from serial import Serial

running = True

MOVE_LEFT = b'\x00'
MOVE_RIGHT = b'\x01'
GOTO_SAFEGUARD = b'\x02'
NORMAL_OPERATION = b'\x03'


class ArduinoCom(Node):
    def __init__(self, baudRate, serialPortName):
        super().__init__('arduino_com')
        self.serialPort = self.setupSerial(baudRate, serialPortName)
        self.marker = '\n'
        self.waitForArduino()
        self.rotation_sub = self.create_subscription(
            Rotate,
            'rotate_dir',
            self.rotation_callback,
            10,
        )
        self.mode_sub = self.create_subscription(
            Mode,
            'set_mode',
            self.mode_callback,
            10,
        )

        self.reader = Thread(target=reader, args=[self])
        self.reader.start()

    def rotation_callback(self, msg):
        if msg.dir > 0:
            print('Rotando <-')
            self.serialPort.write(MOVE_LEFT)
        else:
            print('Rotando ->')
            self.serialPort.write(MOVE_RIGHT)

    def mode_callback(self, msg):
        if msg.mode == 'safeguard':
            print('Yendo a safeguard')
            self.serialPort.write(GOTO_SAFEGUARD)
        elif msg.mode == 'normal':
            print('Yendo a operación normal')
            self.serialPort.write(NORMAL_OPERATION)

    def setupSerial(self, baudRate, serialPortName):
        serialPort = Serial(
            port=serialPortName,
            baudrate=baudRate,
            timeout=0,
            rtscts=True,
        )

        print(f'Puerto serial {serialPortName} abierto')

        return serialPort

    def readMsg(self):
        global running
        buffer = ''
        while running:
            if self.serialPort.in_waiting > 0:
                b = self.serialPort.read().decode('utf-8')
                if b == self.marker:
                    return buffer

                buffer += b

    def waitForArduino(self):
        print('Esperando reset de arduino')
        msg = ''
        while msg.find('Arduino is ready') == -1:
            msg = self.readMsg()
            print(msg)

        print('Arduino inicializado')

    def stop(self):
        self.running = False


def reader(serialPort):
    global running
    while running:
        print(serialPort.readMsg())


def main(args=None):
    rclpy.init(args=args)
    arduinoCom = ArduinoCom(115200, '/dev/ttyACM2')

    rclpy.spin(arduinoCom)
    arduinoCom.stop()

    arduinoCom.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
