import rclpy
from rclpy.node import Node

from ecosun_msgs.msg import Mode, Position


class EcosunListener(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
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

    def mode_callback(self, msg):
        print(f'Modo de operación: {msg.mode}')

    def position_callback(self, msg):
        print(f'Posición actual: {msg.position}')


def main(args=None):
    rclpy.init(args=args)

    print('Escuchando por modo y posición')

    minimal_subscriber = EcosunListener()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
