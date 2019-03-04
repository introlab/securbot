module.exports.clearLegend = function() {

    // Selecting Burndown chart legend
    var bdFrame = document.getElementById("gadget-10101");
    var legend = bdFrame.contentDocument.querySelector("#ghx-chart-view>div.legend>table");

    // Adjusting position
    legend.style.removeProperty("top");
}
