document.getElementById("cancel").addEventListener("click", async(event) => {
    event.preventDefault();

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