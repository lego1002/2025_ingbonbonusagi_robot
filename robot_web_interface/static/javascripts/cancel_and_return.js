document.getElementById("cancel").addEventListener("click", async(event) => {
    event.preventDefault();
    
    fetch("http://127.0.0.1:5000/stop-program")
        .then(res => res.json())
        .then(data => console.log(data));
    
    try {
        await fetch("/set_state", {
            method: "POST", 
            headers: {"Content-Type": "application/json"},
            body: JSON.stringify({
                states: {
                    1: 0,
                    2: 0,
                    3: 0
                }
            })
        });
    } catch (error) {
        console.error("Failed to reset states: ", error);
    }
    
    const homePage = '/';
    window.location.href= homePage;
})