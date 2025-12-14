let colors = ["white", "red", "limegreen"];
let interval = 500;

let boxes = [
    {element: document.getElementById("bar1"), colors, interval, isFirstColor: true, timerID: null},
    {element: document.getElementById("bar2"), colors, interval, isFirstColor: true, timerID: null},
    {element: document.getElementById("bar3"), colors, interval, isFirstColor: true, timerID: null}
]

let messages = ["Order Received!!!", "Order Prepared!!!", "Order Delivered!!!"];

let textMessage = [
    {element: document.getElementById("mess1"), messages},
    {element: document.getElementById("mess2"), messages},
    {element: document.getElementById("mess3"), messages}
]


//funciton definitions
function blinking(index) {
    let box = boxes[index];
    if (box.timerID !== null) return;

    box.timerID = setInterval(() => {
        box.element.style.backgroundColor = box.isFirstColor ? box.colors[1] : box.colors[0];
        box.isFirstColor = !box.isFirstColor;
    }, box.interval);
}

function stopBlinking(index) {
    let box = boxes[index];
    if (box.timerID !== null) {
        clearInterval(box.timerID);
        box.timerID = null;
    }
}

function resetBlinkingBox(index) {
    stopBlinking(index);
    let box = boxes[index];
    box.isFirstColor = true;
    box.element.style.backgroundColor = box.colors[2];
    

    let text = textMessage[index];
    text.element.innerText = text.messages[index];
}

async function updateBoxStateFromServer() {
    try {                                           // locally handle errors with try-catch
        const response = await fetch('http://127.0.0.1:5000/get_states');
        const data = await response.json();

        boxes.forEach((box, index) => {
            const state = data[index + 1];
            if (state === 0) {
                // do nothing: zero is the initial state
            } else if (state === 1) {
                blinking(index);
            } else {
                resetBlinkingBox(index);
            }
        });
    } catch (error) {
        console.error("Error fetching box states:", error);
    }
}

// control programs are down here
setInterval(updateBoxStateFromServer, 500);