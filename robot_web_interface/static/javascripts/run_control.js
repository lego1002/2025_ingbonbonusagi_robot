document.querySelectorAll(".bev-button").forEach(button => {
    button.addEventListener("click", (event) => {
        event.preventDefault();
        const boxId = button.dataset.boxId;
        const nextPage = '/progress' + boxId;
        fetch("http://127.0.0.1:5000/start-program", {           // send a POST request to the flask server
            method: "POST"                                       //there must be a "/start-program" route defined in the flask server (user_interface.py)
        })
        window.location.href = nextPage;
    });
})