module.exports = function({width = "80px", height = "80px", paddingBottom = "10px"} = {}) {
    let gadget = document.getElementById("iframe-gadget")
    let pictures = gadget.contentDocument.getElementsByClassName("sc-elJkPf lhRrZk")

    console.log("Enlarging...")

    try {
        for(let pic of pictures) {

            pic.style.width = width
            pic.style.height = height

            let child = pic.firstChild
            let background = child.style.backgroundImage
            let updatedBackground = background.replace("medium", "xxxl")
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
