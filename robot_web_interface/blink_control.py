import requests

requests.post("http://127.0.0.1:5000/set_state", json={"box_id": 1, "value": 2})
requests.post("http://127.0.0.1:5000/set_state", json={"box_id": 2, "value": 1})
print("Preparing Beverage (Box 2 is blinking)")

a = input(": ")

if a == 'finished':
    requests.post("http://127.0.0.1:5000/set_state", json={"box_id": 2, "value": 2})
    requests.post("http://127.0.0.1:5000/set_state", json={"box_id": 3, "value": 1})
    print("Delivering (Box 3 is blinking)")
    b = input(": ")

    if b == "delivered":
            requests.post("http://127.0.0.1:5000/set_state", json={"box_id": 3, "value": 2})