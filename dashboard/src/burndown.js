module.exports.clearLegend = function() {

    // Selecting Burndown chart legend
    var bdFrame = document.getElementById("gadget-10101");
    var legend = bdFrame.contentDocument.querySelector("#ghx-chart-view>div.legend>table");
    var legendBackground = bdFrame.contentDocument.querySelector("#ghx-chart-view>div.legend>div");

    // Adjusting position
    legend.style.removeProperty("top");
    legendBackground.style.removeProperty("top");
}
