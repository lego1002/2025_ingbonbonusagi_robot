import requests
import time

time.sleep(2)
requests.post("http://127.0.0.1:5000/set_state", json={"box_id": 1, "value": 2})
requests.post("http://127.0.0.1:5000/set_state", json={"box_id": 2, "value": 1})

time.sleep(10)
requests.post("http://127.0.0.1:5000/set_state", json={"box_id": 2, "value": 2})
requests.post("http://127.0.0.1:5000/set_state", json={"box_id": 3, "value": 1})

time.sleep(10)
requests.post("http://127.0.0.1:5000/set_state", json={"box_id": 3, "value": 2})