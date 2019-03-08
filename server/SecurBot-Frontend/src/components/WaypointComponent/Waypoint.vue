<template>
  <div id="patrol-container">
    <div class="list-container">
      <button class="btn" v-on:click=sendPatrol()>Confirm</button>
      <button class="btn" v-on:click=clearWaypointList()>Reset</button>
      <table id="waypoint-table" class="waypoint-list"></table>
    </div>
    <div class="map-container">
      <video-box :VideoId="mapId" :show="showMap" class="map-video"/>
      <canvas ref="canvas" class="map-canvas"
      @mousedown="onMouseDown"/>
    </div>  
  </div>
</template>

<script>
/*
  Still to be done:
    Fix the names and calls to fit standards
    Program the sendPatrol function
    Add some CSS (table row background, border, etc)
    Fix the fact that waypoints are not shown on map
    Add comment (Documentation)
    Find images to use for some control
*/
import VideoBox from '../VideoComponent/VideoBox.vue'
export default {
  name: 'waypoint',
  props: ['mapId','showMap'],
  components: {
    VideoBox
  },
  data(){
    return{
      videoElement: null,
      canvas: null,
      loopIntervalId: null,
      enable: false,
      CanvasRefreshRate: 60.0, //Hz
      context: null,
      waypointList: [],
      index: null,
    }
  },
  methods: {
    init() {
      this.loopIntervalId = setInterval(function() {
        if (this.enable) {
          this.adjustCanvasToVideo();
          this.drawCanvas();
        }
      }.bind(this), 1000 / this.CanvasRefreshRate);
    },

    // Setup of canvas and video 
    drawCanvas() {
      this.context.clearRect(0, 0, this.canvas.width, this.canvas.height);
    },
    adjustCanvasToVideo() {
      this.canvas.width = this.videoElement.offsetWidth;
      this.canvas.height = this.videoElement.offsetHeight;
    },
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
    // END of Setup of canvas and video 

    //Setup of waypoint list
    addWaypoint(WP) {
      this.waypointList.push(WP);
      this.addWaypointToList(WP);
      this.displayWaypoints(WP);
    },
    clearWaypointList(){
      this.waypointList = [];
      var table = document.getElementById("waypoint-table");
      table.innerHTML =  "";
      this.setTableHeader();
    },
    setTableHeader(){
      var table = document.getElementById("waypoint-table");
      var header = table.createTHead();

      //Create elements
      var row = header.insertRow(0);
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
    addWaypointToList(WL){
      var table = document.getElementById("waypoint-table");
      var lengthTable = table.rows.length;
      //Create delete button
      var removeBtn = document.createElement("button");
      removeBtn.id = "removeBtn-"+lengthTable;
      removeBtn.className = "removeBtn";
      removeBtn.onclick = function(){this.removeWaypointFromList(lengthTable);}.bind(this);   
      //
      var row = table.insertRow(lengthTable);
      var cell1 = row.insertCell(0);
      var cell2 = row.insertCell(1);
      var cell3 = row.insertCell(2);
      var cell4 = row.insertCell(3);
      var cell5 = row.insertCell(4);
      cell1.innerHTML = lengthTable;
      cell2.innerHTML = WL.coordX.toFixed(1);
      cell3.innerHTML = WL.coordY.toFixed(1);
      cell4.innerHTML = WL.orient.toFixed(1);
      cell5.appendChild(removeBtn);
    },
    removeWaypointFromList(index){
      this.waypointList.splice(index-1,1);
      var table = document.getElementById("waypoint-table");
      var lengthTable = table.rows.length;
      var lengthWaypoints = this.waypointList.length;
      for(var i = index; i < lengthTable; i++)
      {
        var row = table.deleteRow(index);
      };
      
      for(var i = index-1; i < lengthWaypoints; i++)
      { 
        var waypoint={};
        waypoint.coordX = this.waypointList[i].coordX;
        waypoint.coordY = this.waypointList[i].coordY;
        waypoint.orient = this.waypointList[i].orient;
        this.addWaypointToList(waypoint);
      };   
    },
    //END of Setup of waypoint list

    //Display of waypointslet goalColor = '#00FF00';
    displayWaypoints(WP){
      let WPColor = "00FF00";
      let canvasWP = this.getCanvasCoordinatesFromVideo(WP.x, WP.y);

      //Draw the goal circle
      let WPRadius = Math.max(10, 8 / 8);

      this.context.beginPath();
      this.context.arc(canvasWP.x, canvasWP.y, WPRadius, 0, 2 * Math.PI);
      this.context.fillStyle = WPColor;
      this.context.fill();
    },
    //END of display waypoints

    //Get position from click
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
    getCanvasCoordinatesFromVideo(x, y) {
      let offsetAndScale = this.getVideoOffsetAndScale();

      return {
        x: x * offsetAndScale.scale + offsetAndScale.offsetX,
        y: y * offsetAndScale.scale + offsetAndScale.offsetY
      };
    }, 
    //END Get position on click

    //On mouse event
    onMouseDown(event) {
      if (event.button === 0) {
        let WPclick = this.getVideoCoordinatesFromEvent(event);
        if (this.isClickValid(WPclick)) {
          var waypoint={};
          waypoint.coordX = WPclick.x;
          waypoint.coordY = WPclick.y;
          waypoint.orient = 0;
          this.addWaypoint(waypoint);
        }
      }
    },
    isClickValid(WPclick) {
      return WPclick.x >= 0 &&
        WPclick.x < this.videoElement.videoWidth &&
        WPclick.y >= 0 &&
        WPclick.y < this.videoElement.videoHeight; 
    },
    //END on mouse event
    sendPatrol(){
      console.log("I should be sending the patrol but im not :^)");
    }
  },
  mounted() {
    this.videoElement = document.getElementById(this.mapId);
    this.canvas = this.$refs.canvas;
    this.context = this.canvas.getContext('2d');
    this.init();
    this.setTableHeader();
  },
  destroyed() {
    clearInterval(this.loopIntervalId);
  }
};
</script>

<style>
#patrol-container{
  width: 100%;
  height: 100%;
}
.map-video {
  width: 100%;
  height: 100%;
  position: absolute;
  top: 0;
  left: 0;
}
.map-canvas {
  width: 100%;
  height: 100%;
  position: absolute;
  top: 0;
  left: 0;
  z-index: 10;
}
.list-container{
  width: 18%;
  height: 100%;
  position: relative;
  float: left;
}
.map-container{
  width: 80%;
  height: 100%;
  position: relative;
  float: right;
}
.btn{
  width: 80px;
  height: 20px;
}
.removeBtn{
  width: 20px;
  height: 20px;
  background-color: darkred;
}
.waypoint-cell{
  width: 20%;
}
</style>