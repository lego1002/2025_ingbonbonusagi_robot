import os
from flask import Flask, render_template, jsonify, request

def get_package_share_directory(package_name: str) -> str:
    """
    Find the ROS 2 package share directory at runtime.
    Works for ros2 run and ros2 launch.
    """
    ament_prefixes = os.environ.get('AMENT_PREFIX_PATH', '').split(':')
    for prefix in ament_prefixes:
        candidate = os.path.join(prefix, 'share', package_name)
        if os.path.isdir(candidate):
            return candidate
    raise RuntimeError(f'Package share directory not found for {package_name}')

def create_app(node):
    pkg_share = get_package_share_directory('flask_ui')

    static_dir = os.path.join(pkg_share, 'static')
    template_dir = os.path.join(pkg_share, 'templates')

    app = Flask(
        __name__,
        template_folder=template_dir,
        static_folder=static_dir
    )

    @app.route("/")
    def index():
        return render_template("base.html")

    @app.route("/progress<int:box_id>")
    def progress(box_id):
        return render_template(
            "progress.html",
            page=f"Beverage {box_id}",
            box_id=box_id
        )

    @app.route("/get_states")
    def get_states():
        return jsonify(node.box_states)

    @app.route("/set_state", methods=["POST"])
    def set_state():
        data = request.get_json()

        if "states" in data:
            for box_id, value in data["states"].items():
                node.box_states[int(box_id)] = int(value)
            return jsonify({"status": "success"})

        box_id = data.get("box_id")
        value = data.get("value")
        node.box_states[int(box_id)] = int(value)
        return jsonify({"status": "success"})

    @app.route("/start_moving", methods=["POST"])
    def start_moving():
        data = request.get_json()
        node.publish_box(data["box_id"])
        return jsonify({"status": "received"})

    return app