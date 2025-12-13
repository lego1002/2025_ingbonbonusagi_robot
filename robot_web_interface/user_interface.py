import rclpy
from rclpy.node import Node
from threading import Thread 
from flask import Flask, render_template, jsonify, request  
from std_msgs.msg import Int32MultiArray, String

app = Flask(__name__)                   # create Flask app instance (server)

class FlaskNode(Node):
    def __init__(self):
        super().__init__('flask_node')
        self.get_logger().info('Flask Node has been started.')
        self.publisher = self.create_publisher(Int32MultiArray, 'traj_point', 10)
        
        # box states dictionary, key [1, 2, 3] = box number, 
        # value [0, 1, 2] = [initial state, blinking on, blinking off]
        self.box_states = {                          
            1: 0,
            2: 0,
            3: 0
        }
        
        flask_thread = Thread(target=self.run_flask)
        flask_thread.start()
        
    def publish_box(self, box_id):
        msg = Int32MultiArray()
        
        #ONLY FOR TESTING PURPOSES
        if box_id == '1':
            msg.data = [1, 2]
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Published command: {box_id}')
    
        
    def run_flask(self):        
        # HTML route definitions
        @app.route("/")
        def hello_world():                                   # define the function for homepage's route
            return render_template('base.html')
        
        @app.route('/progress<int:box_id>')                  # dynamically render progress page based on box_id
        def progress(box_id):
            return render_template('progress.html', page=f'Beverage {box_id}', box_id=box_id)

        # API route definitions
        @app.route('/get_states', methods=['GET'])           # When JS makes a GET request to /get_states
        def get_states():
            return jsonify(self.box_states)                  # convert box_states dictionarty to JSON and return it for JS to process

        @app.route('/set_state', methods=['POST'])           # JS makes a POST request to /set_state
        def set_state():
            data = request.get_json()                        # get the JSON data sent from JS    
            # Bulk state update
            if "states" in data:
                for box_id, value in data["states"].items():
                    box_id = int(box_id)
                    if box_id in self.box_states and value in [0, 1, 2]:
                        self.box_states[box_id] = int(value)
                return jsonify({"status": "success", "states": self.box_states})

            # Single state update
            box_id = data.get('box_id')
            value = data.get('value')
            if box_id in self.box_states and value in [0, 1, 2]:    # validate box_id and value before updating and replying to JS
                self.box_states[box_id] = value
                return jsonify({"status": "success", "box_id": box_id, "value": value})
            else:
                return jsonify({"status": "error"}), 400
            
        @app.route('/start_moving', methods=['POST'])
        def start_moving():
            data = request.get_json()
            box_id = data.get('box_id')
            if not box_id:
                return jsonify({"status": "error"}), 400
            self.publish_box(box_id)
            return jsonify({"status": "received", "beverage": box_id})

        app.run(host="127.0.0.1", port=5000, debug=False, use_reloader=False)


def main(args=None):
    rclpy.init(args=args)
    flask_node = FlaskNode()
    try:
        rclpy.spin(flask_node)
    except KeyboardInterrupt:
        pass
    finally:
        flask_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()