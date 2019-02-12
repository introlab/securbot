const enlarge = function(options) {
    let gadget = document.getElementById("iframe-gadget")
    let pictures = gadget.contentDocument.getElementsByClassName("dmmjUH")

    console.log("Enlarging...")

    try {
        for(let pic of pictures) {

            pic.style.width = options.width
            pic.style.height = options.height

            let child = pic.firstChild
            let background = child.style.backgroundImage
            let updatedBackground = background.replace("medium", "large")
            child.style.backgroundImage = updatedBackground

            let parent = pic.parentNode
            parent.style.width = options.width
            parent.style.height = options.height
            parent.style.paddingBottom = options.paddingBottom

            parent = parent.parentNode
            parent.style.flexDirection = "column"
        }
    }

    catch (err) {
        console.log(err)
    }
}

enlarge({width: "80px", height: "80px", paddingBottom = "10px"});