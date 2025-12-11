from flask import Flask, render_template, jsonify, request  
from flask_cors import CORS             # enable CORS for cross-origin requests
import subprocess
import sys

app = Flask(__name__)                   # create Flask app instance (server)
CORS(app)                               # enable CORS on the app      

box_states = {                          # box states dictionary, number [1, 2, 3] = box number, [0, 1, 2] = [initial state, blinking on, blinking off]
    1: 0,
    2: 0,
    3: 0
}

# HTML route definitions
@app.route("/")
def hello_world():                      # define the function for homepage's route
    return render_template('base.html')

@app.route('/progress<int:box_id>')     # dynamically render progress page based on box_id
def progress(box_id):
    return render_template('progress.html', page=f'Beverage {box_id}', box_id=box_id)


# API route definitions
@app.route('/get_states', methods=['GET'])      # When JS makes a GET request to /get_states
def get_states():
    return jsonify(box_states)                  # convert box_states dictionarty to JSON and return it for JS to process

@app.route('/set_state', methods=['POST'])      # JS makes a POST request to /set_state
def set_state():
    data = request.json                         # get the JSON data sent from JS    

    if "states" in data:
        for box_id, value in data["states"].items():
            box_id = int(box_id)
            if box_id in box_states and value in [0, 1, 2]:
                box_states[box_id] = int(value)
        return jsonify({"status": "success", "states": box_states})

    box_id = data.get('box_id')
    value = data.get('value')
    if box_id in box_states and value in [0, 1, 2]:    # validate box_id and value before updating and replying to JS
        box_states[box_id] = value
        return jsonify({"status": "success", "box_id": box_id, "value": value})
    else:
        return jsonify({"status": "error"}), 400

@app.route('/start-program', methods=['POST'])          # define a "/start-program" route to receive POST requests from JS
def start_program():
    process = subprocess.Popen(
        [sys.executable, 'test_button_function.py'],      # 2025/12/11: changed from 'python' to 'sys.executable' for Windows compatibility
        #['python', 'test_button_function.py'],     # start the external python script as a separate process, meaning Flask server contiues to run
        stdout = subprocess.PIPE,
        stderr = subprocess.PIPE,
        text = True
    )
    out, err = process.communicate()
    return jsonify({                                # respond to JS with a JSON message, this will later be logged to browser console
        "status": "Program Started",
        "output": out,
        "error": err
        })      

if __name__ == "__main__":
    app.run(debug=True)