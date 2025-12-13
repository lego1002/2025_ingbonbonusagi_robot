from flask import Flask, render_template, jsonify, request

def create_app(node):
    app = Flask(
        __name__,
        template_folder="templates",
        static_folder="static"
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