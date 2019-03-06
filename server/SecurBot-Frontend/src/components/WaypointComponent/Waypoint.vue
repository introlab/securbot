<template>
  <div>
    <div>
      <video-box :VideoId="MapId" :show="ShowMap" class="map-video"/>
      <canvas ref="canvas" class="map-canvas"
      @mousedown="onMouseDown" 
      />
    </div>  
    <div>
      <table id="waypoint-list" class="waypoint-list"></table>
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
      Waypoint: null,
      WaypointList: [],
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
      this.WaypointList.push = this.WP;
      this.updateWaypointList(this.WP);
      this.displayWaypoints(this.WP);
    },
    clearWaypointList(){
      this.WaypointList = "";
      var Table=document.getElementById("waypoint-list");
      Table.innerHTML =  "";
      setTableHeader();
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
      cell2.innerHTML = "coordX";
      cell3.innerHTML = "coordY";
      cell4.innerHTML = "orient";
      cell5.innerHTML = "Delete waypoint";
    },
    updateWaypointList(WL){
      var Table=document.getElementById("waypoint-list");
      var lengthTable= Table.rows.length;

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
      cell5.innerHTML = "byebye";
    },
    //END of Setup of waypoint list

    //Display of waypointslet goalColor = '#00FF00';
    displayWaypoints(WP){
      let WPColor = '#00FF00';
      let canvasWP = this.getCanvasCoordinatesFromVideo(WP.x, WP.y);

      //Draw the goal circle
      let WPRadius = Math.max(1, 8 / 8)

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
        console.log("yo");
        if (this.isClickValid(WPclick)) {
          console.log("hey");
          this.Waypoint.coordX = WPclick.x;
          this.Waypoint.coordY = WPclick.y;
          this.Waypoint.orient = 0;
          addWaypoint(this.Waypoint);
        }
      }
    },
    isClickValid(WPclick) {
      console.log(WPclick);
      return WPclick.x >= 0 &&
        WPclick.x < this.videoElement.videoWidth &&
        WPclick.y >= 0 &&
        WPclick.y < this.videoElement.videoHeight; 
    }
    //END on mouse event
  },
  mounted() {
    this.videoElement = document.getElementById(this.MapId);
    this.canvas = this.$refs.canvas;
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
</style>