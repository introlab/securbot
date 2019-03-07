<template>
  <div>
    <div class="waypoint-list">
      <video-box :VideoId="MapId" :show="ShowMap" class="map-video"/>
      <canvas ref="canvas" class="map-canvas"
      @mousedown="onMouseDown"/>
      
    </div>  
    <div>
      <table id="waypoint-list" class="waypoint-list"></table>
      <button class="button" v-on:click=clearWaypointList()>Clear all</button>
    </div>
  </div>
</template>

<script>
import VideoBox from '../VideoComponent/VideoBox.vue'
export default {
  name: 'waypoint',
  props: ['MapId','ShowMap'],
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
      WaypointList: [],
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
      this.WaypointList.push(WP);
      this.addWaypointToList(WP);
      this.displayWaypoints(WP);
    },
    clearWaypointList(){
      this.WaypointList = [];
      var Table=document.getElementById("waypoint-list");
      Table.innerHTML =  "";
      this.setTableHeader();
    },
    setTableHeader(){
      var Table=document.getElementById("waypoint-list");
      var header = Table.createTHead();

      var row = header.insertRow(0);
      var cell1 = row.insertCell(0);
      var cell2 = row.insertCell(1);
      var cell3 = row.insertCell(2);
      var cell4 = row.insertCell(3);
      var cell5 = row.insertCell(4);
      cell1.innerHTML = "Waypoint number";
      cell2.innerHTML = "Coordinate X";
      cell3.innerHTML = "Coordinate Y";
      cell4.innerHTML = "Orientation";
      cell5.innerHTML = "Delete waypoint";
    },
    addWaypointToList(WL){
      var Table=document.getElementById("waypoint-list");
      var lengthTable= Table.rows.length;
      //Create delete button
      var trash = document.createElement("button");
      trash.id="button_"+lengthTable;
      trash.className="trash-can";
      trash.onclick= function(){this.removeWaypointFromList(lengthTable);}.bind(this);   
      //
      var row = Table.insertRow(lengthTable);
      var cell1 = row.insertCell(0);
      var cell2 = row.insertCell(1);
      var cell3 = row.insertCell(2);
      var cell4 = row.insertCell(3);
      var cell5 = row.insertCell(4);
      cell1.innerHTML = lengthTable;
      cell2.innerHTML = WL.coordX;
      cell3.innerHTML = WL.coordY;
      cell4.innerHTML = WL.orient;
      cell5.appendChild(trash);
    },
    removeWaypointFromList(index){
      this.WaypointList.splice(index-1,1);
      var Table=document.getElementById("waypoint-list");
      var lengthTable= Table.rows.length;
      var lengthWaypoints = this.WaypointList.length;
      //console.log("removeWP lengthTable "+lengthTable+" lenghtWP "+lengthWaypoints);
      var i;
      for(i=index;i<lengthTable;i++)
      {
        var row = Table.deleteRow(index);
      };
      //console.log(index);
      
      for(i=index;i<lengthWaypoints;i++)
      { 
        //console.log(i);
        var waypoint={};
        waypoint.coordX = this.WaypointList[i].coordX;
        waypoint.coordY = this.WaypointList[i].coordY;
        waypoint.orient = this.WaypointList[i].orient;
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
  },
  mounted() {
    this.videoElement = document.getElementById(this.MapId);
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
.waypoint-list{
  width: 100%;
  height: 100%;
  position: relative;
}
.half-height{
  width: 100%;
  height: 50%;
}
.button{
  width: 80px;
  height: 20px;
}
.trash-can{
  width: 20px;
  height: 20px;
}
</style>