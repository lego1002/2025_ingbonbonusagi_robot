import rclpy
from rclpy.node import Node
from threading import Thread
from std_msgs.msg import Int32MultiArray

from flask_ui.web import create_app


class FlaskNode(Node):
    def __init__(self):
        super().__init__("flask_node")

        self.publisher = self.create_publisher(
            Int32MultiArray,
            "traj_point",
            10
        )

        self.box_states = {
            1: 0,
            2: 0,
            3: 0
        }

        app = create_app(self)

        Thread(
            target=app.run,
            kwargs={
                "host": "127.0.0.1",
                "port": 5000,
                "use_reloader": False
            },
            daemon=True
        ).start()

        self.get_logger().info("Flask UI started")

    def publish_box(self, box_id):
        msg = Int32MultiArray()
        msg.data = [int(box_id)]
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FlaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()