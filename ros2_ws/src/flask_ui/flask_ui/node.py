import rclpy
from rclpy.node import Node
from threading import Thread
from std_msgs.msg import Int32MultiArray

from flask_ui.web import create_app


class FlaskNode(Node):
    # 4 sets of predefined trajectory points for 4 boxes (beverage options)
    traj_point = {
        '1': [0, 1],
        '2': [3, 2],
        '3': [2, 1],
        '4': [0, 2]
    }
    
    def __init__(self):
        super().__init__("flask_node")
        self.get_logger().info("Starting Flask UI...")

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
                "use_reloader": False,
                "debug": False
            },
            daemon=True
        ).start()

        self.get_logger().info("Flask UI started")

    def publish_box(self, box_id):
        msg = Int32MultiArray()
        buttonNum = int(box_id)
        msg.data = self.traj_point[str(buttonNum)]
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = FlaskNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass    
    node.destroy_node()
    rclpy.shutdown()