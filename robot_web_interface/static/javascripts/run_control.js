document.querySelectorAll(".bev-button").forEach(button => {
    button.addEventListener("click", (event) => {
        event.preventDefault();
        const boxId = button.dataset.boxId;
        const nextPage = '/progress' + boxId;
        fetch("http://127.0.0.1:5000/start-program", {           // send a POST request to the flask server
            method: "POST"                                       //there must be a "/start-program" route defined in the flask server (user_interface.py)
        })

        //wait for the response from flask server and parse it as JSON
        .then(res => res.json())                                 
        .then(data => {
        //print (log) the response to browser console
        console.log("Python output: ", data.output); 

        //redirect to the next page after logging the response
        window.location.href = nextPage;
        })
        .catch(err => console.error("Fech error: ", err));
    });
})