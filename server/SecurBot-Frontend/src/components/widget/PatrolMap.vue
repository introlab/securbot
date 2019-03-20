<template>
  <div class="h-100 w-100 position-relative">
    <video-box VideoId="map" :show="true" class="w-100 h-100 position-absolute" style="top:0;left:0;"/>
    <canvas ref="canvas" class="w-100 h-100 position-absolute" style="top:0;left:0;z-index:10;"
    @mousedown="onMouseDown"/>
  </div>
</template>

<script>
/*
* Author(s):  Edouard Legare <edouard.legare@usherbrooke.ca>,
*             Valerie Gauthier <valerie.gauthier4@usherbrooke.ca>
* File :  PatrolMap.vue
* Desc :  Vue SFC used as a widget to set waypoint on a map. This component
*         uses the VideoBox component to show the robot map (that is sent from
*         the robot in a video feed format) and put a canvas on top of it to 
*         detect user clicks. When a click is detected the x and y position 
*         is use to set a waypoint in the array given in props (push). The
*         canvas then read the array and draw waypoint on the map where the 
*         user clicked previously. 
*
* Dependencies : 
*       -VideoBox.vue
*       -Bootstrap-Vue
*
*/

import VideoBox from './VideoBox.vue'

export default {
  name:'patrol-map',
  props:['waypointList'],
  components:{
    VideoBox
  },
  data(){
    return{
      videoElement: null,
      canvas: null,
      context: null,
      CanvasRefreshRate: 60.0, //Hz
      loopIntervalId:null,
      enable:true,
    }
  },
  methods: {
    //Initialisation of canvas refrash rate
    init() {
      this.loopIntervalId = setInterval(function() {
        if (this.enable) {
          this.adjustCanvasToVideo();
          this.drawCanvas();
        }
      }.bind(this), 1000 / this.CanvasRefreshRate);
    },
    //Add waypoint to the list
    addWaypoint(wp) {
      this.waypointList.push(wp);
    },
    //Clean canvas and redraw the waypoints
    drawCanvas() {
      this.context.clearRect(0, 0, this.canvas.width, this.canvas.height);
      this.drawWaypoints()
    },
    //Draw waypoints on canvas
    drawWaypoints(){
      this.waypointList.forEach(function(wp){
        let wpColor = "#00FF00";
        let coord = this.getCanvasCoordinatesFromVideo(wp.x, wp.y);

        //Draw the goal circle
        let wpRadius = 8;

        this.context.beginPath();
        this.context.arc(coord.x, coord.y, wpRadius, 0, 2 * Math.PI); 
        this.context.fillStyle = wpColor;
        this.context.fill();
        
      }.bind(this));
    },    
    //Get position/coordinate of video on click
    getVideoCoordinatesFromEvent(event) {
      let offsetAndScale = this.getVideoOffsetAndScale();

      let rect = this.videoElement.getBoundingClientRect();
      let x = (event.clientX - rect.left - offsetAndScale.offsetX) / offsetAndScale.scale;
      let y = (event.clientY - rect.top - offsetAndScale.offsetY) / offsetAndScale.scale;
      return {
        x: x,
        y: y
      };
    },    
    //Ajust canvas size to fit video
    adjustCanvasToVideo() {
      this.canvas.width = this.videoElement.offsetWidth;
      this.canvas.height = this.videoElement.offsetHeight;
    },
    //Get the offset and scale of canvas
    getVideoOffsetAndScale() {
      let videoRatio = this.videoElement.videoWidth / this.videoElement.videoHeight;

      let offsetX = 0;
      let offsetY = 0;
      let scale = 1;
      if ((this.videoElement.offsetHeight * videoRatio) > this.videoElement.offsetWidth) {
        scale = this.videoElement.offsetWidth / this.videoElement.videoWidth;
        offsetY = (this.videoElement.offsetHeight - this.videoElement.videoHeight * scale) / 2;
      }
      else {
        scale = this.videoElement.offsetHeight / this.videoElement.videoHeight;
        offsetX = (this.videoElement.offsetWidth - this.videoElement.videoWidth * scale) / 2;
      }
      return {
        offsetX: offsetX,
        offsetY: offsetY,
        scale: scale
      }
    },
    //Set html table header - NOT USED, NEED TO BE REMOVE
    setTableHeader(){
      var table = document.getElementById("waypoint-table");
      var header = table.createTHead();

      //Create elements
      var row = header.insertRow(0);
      row.className = "table-header";
      var cell1 = row.insertCell(0);
      var cell2 = row.insertCell(1);
      var cell3 = row.insertCell(2);
      var cell4 = row.insertCell(3);
      var cell5 = row.insertCell(4);

      //Give class to the cells
      cell1.className = "waypoint-cell";
      cell2.className = "waypoint-cell";
      cell3.className = "waypoint-cell";
      cell4.className = "waypoint-cell";
      cell5.className = "waypoint-cell";

      //Set value of cells
      cell1.innerHTML = "#";
      cell2.innerHTML = "X";
      cell3.innerHTML = "Y";
      cell4.innerHTML = "Yaw";
      cell5.innerHTML = "Remove";
    },
    //Add waypoint to html table - NOT USED, NEED TO BE REMOVE
    addWaypointToList(wp){
      //Get table
      var table = document.getElementById("waypoint-table");
      var lengthTable = table.rows.length;
      //Create remove button
      var removeBtn = document.createElement("button");
      removeBtn.id = "removeBtn-"+lengthTable;
      removeBtn.className = "removeBtn";
      removeBtn.onclick = function(){this.removeWaypointFromList(lengthTable);}.bind(this);   
      //Create and set row
      var row = table.insertRow(lengthTable);
      row.className = "table-row";
      //Add a cells in row
      var cell1 = row.insertCell(0);
      var cell2 = row.insertCell(1);
      var cell3 = row.insertCell(2);
      var cell4 = row.insertCell(3);
      var cell5 = row.insertCell(4);
      //Set the cells content
      cell1.innerHTML = lengthTable;
      cell2.innerHTML = wp.x.toFixed(1);
      cell3.innerHTML = wp.y.toFixed(1);
      cell4.innerHTML = wp.yaw.toFixed(1);
      cell5.appendChild(removeBtn);
    },
    //Make correction to the coordinates from canvas for the waypoint given 
    getCanvasCoordinatesFromVideo(x, y) {
      let offsetAndScale = this.getVideoOffsetAndScale();

      return {
        x: x * offsetAndScale.scale + offsetAndScale.offsetX,
        y: y * offsetAndScale.scale + offsetAndScale.offsetY
      };
    }, 
    //On mouse down event, verify validity of click
    onMouseDown(event) {
      if (event.button === 0) {
        let coord = this.getVideoCoordinatesFromEvent(event);
        if (this.isClickValid(coord)) {
          var wp = coord;
          wp.yaw = 0;
          this.addWaypoint(wp);
        }
      }
    },
    //Check is the click was inbound
    isClickValid(coord) {
      return coord.x >= 0 &&
        coord.x < this.videoElement.videoWidth &&
        coord.y >= 0 &&
        coord.y < this.videoElement.videoHeight; 
    },
  },
  //On component mounted, Get html elements and initialize
  mounted() {
    this.videoElement = document.getElementById('map');
    this.canvas = this.$refs.canvas;
    this.context = this.canvas.getContext('2d');
    this.init();
    //this.setTableHeader();
  },
  //On component destroyed, clear refresh rate of canvas
  destroyed() {
    clearInterval(this.loopIntervalId);
  }
}
</script>

<style>
</style>

