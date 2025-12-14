document.querySelectorAll(".bev-button").forEach(button => {
    button.addEventListener("click", (event) => {
        event.preventDefault();
        
        const boxId = button.dataset.boxId;
        /*const nextPage = '/progress' + boxId;*/

        fetch("http://127.0.0.1:5000/start_moving", {           // send a POST request to the flask server
            method: "POST",                                     //there must be a "/start-program" route defined in the flask server (user_interface.py)
            headers: {                                          // tell the server we are sending JSON data
                "Content-Type": "application/json"
            },
            body: JSON.stringify({                              //send the selected box ID as JSON data (cnovert to string for http transmission) 
                box_id: boxId
            })
        })
        .then(response => response.json())                      //parse the JSON response, after that is redirect       
        /*.then(() => {
            window.location.href = nextPage;
        });*/
    });
})