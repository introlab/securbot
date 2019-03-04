const enlarge = function({width = "80px", height = "80px", paddingBottom = "10px"} = {}) {
    let gadget = document.getElementById("iframe-gadget")
    let pictures = gadget.contentDocument.getElementsByClassName("dmmjUH")

    console.log("Enlarging...")

    try {
        for(let pic of pictures) {

            pic.style.width = width
            pic.style.height = height

            let child = pic.firstChild
            let background = child.style.backgroundImage
            let updatedBackground = background.replace("medium", "large")
            child.style.backgroundImage = updatedBackground

            let parent = pic.parentNode
            parent.style.width = width
            parent.style.height = height
            parent.style.paddingBottom = paddingBottom

            parent = parent.parentNode
            parent.style.flexDirection = "column"
        }
    }

    catch (err) {
        console.log(err)
    }
}

setInterval(enlarge, 2000)