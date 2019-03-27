<template>
  <b-row class="restrict-full-height">
    <b-col
      md="4"
      class="restrict-full-height">
      <div class="patrol-btn-container">
        <div class="title">Patrol :</div>
        <button
          class="btn btn-confirm"
          @click="sendPatrol()">Confirm</button>
        <button
          class="btn btn-cancel"
          @click="clearWaypointList()">Reset</button>
      </div>
      <div class="waypoint-list-container">
        <table
          id="waypoint-table"
          class="waypoint-list"/>
      </div>
    </b-col>
    <b-col
      md="8"
      style="padding:0"
      class="restrict-full-height">
      <div style="heigth:100%;width:100%;margin:auto">
        <video-box
          :video-id="mapId"
          :show="showMap"
          class="map-video"/>
        <canvas
          ref="canvas"
          class="map-canvas"
          @mousedown="onMouseDown"/>
      </div>
    </b-col>
  </b-row >
</template>

<script>
/*
  TO DO:
    Integrate bootstrap and adapt CSS
    Find images to use for some control
*/
import VideoBox from './VideoBox.vue';

export default {
  name: 'waypoint',
  components: {
    VideoBox,
  },
  props: ['mapId', 'showMap', 'bus'],
  data() {
    return {
      videoElement: null,
      canvas: null,
      loopIntervalId: null,
      enable: true,
      CanvasRefreshRate: 60.0, // Hz
      context: null,
      waypointList: [],
      index: null,
    };
  },
  // On component mounted
  mounted() {
    this.videoElement = document.getElementById(this.mapId);
    this.canvas = this.$refs.canvas;
    this.context = this.canvas.getContext('2d');
    this.init();
    this.setTableHeader();
  },
  // On component destroyed
  destroyed() {
    clearInterval(this.loopIntervalId);
  },
  methods: {
    // Initialisation of canvas input
    init() {
      this.loopIntervalId = setInterval(() => {
        if (this.enable) {
          this.adjustCanvasToVideo();
          this.drawCanvas();
        }
      }, 1000 / this.CanvasRefreshRate);
    },
    // Setup for waypoint list
    addWaypoint(wp) {
      this.waypointList.push(wp);
      this.addWaypointToList(wp);
    },
    // Draw the canvas and the waypoints
    drawCanvas() {
      this.context.clearRect(0, 0, this.canvas.width, this.canvas.height);
      this.drawWaypoints();
    },
    // Draw waypoints
    drawWaypoints() {
      this.waypointList.forEach((wp) => {
        const wpColor = '#00FF00';
        const coord = this.getCanvasCoordinatesFromVideo(wp.x, wp.y);

        // Draw the goal circle
        const wpRadius = 8;

        this.context.beginPath();
        this.context.arc(coord.x, coord.y, wpRadius, 0, 2 * Math.PI);
        this.context.fillStyle = wpColor;
        this.context.fill();
      });
    },
    // Get position from canvas on click
    getVideoCoordinatesFromEvent(event) {
      const offsetAndScale = this.getVideoOffsetAndScale();

      const rect = this.videoElement.getBoundingClientRect();
      const x = (event.clientX - rect.left - offsetAndScale.offsetX) / offsetAndScale.scale;
      const y = (event.clientY - rect.top - offsetAndScale.offsetY) / offsetAndScale.scale;
      return {
        x,
        y,
      };
    },
    // Ajust canvas size to fit video
    adjustCanvasToVideo() {
      this.canvas.width = this.videoElement.offsetWidth;
      this.canvas.height = this.videoElement.offsetHeight;
    },
    // Get the offset and scale of canvas
    getVideoOffsetAndScale() {
      const videoRatio = this.videoElement.videoWidth / this.videoElement.videoHeight;

      let offsetX = 0;
      let offsetY = 0;
      let scale = 1;
      if ((this.videoElement.offsetHeight * videoRatio) > this.videoElement.offsetWidth) {
        scale = this.videoElement.offsetWidth / this.videoElement.videoWidth;
        offsetY = (this.videoElement.offsetHeight - this.videoElement.videoHeight * scale) / 2;
      } else {
        scale = this.videoElement.offsetHeight / this.videoElement.videoHeight;
        offsetX = (this.videoElement.offsetWidth - this.videoElement.videoWidth * scale) / 2;
      }
      return {
        offsetX,
        offsetY,
        scale,
      };
    },
    // Clear the list and empty the html table
    clearWaypointList() {
      this.waypointList = [];
      const table = document.getElementById('waypoint-table');
      table.innerHTML = '';
      this.setTableHeader();
    },
    // Set html table header
    setTableHeader() {
      const table = document.getElementById('waypoint-table');
      const header = table.createTHead();

      // Create elements
      const row = header.insertRow(0);
      row.className = 'table-header';
      const cell1 = row.insertCell(0);
      const cell2 = row.insertCell(1);
      const cell3 = row.insertCell(2);
      const cell4 = row.insertCell(3);
      const cell5 = row.insertCell(4);

      // Give class to the cells
      cell1.className = 'waypoint-cell';
      cell2.className = 'waypoint-cell';
      cell3.className = 'waypoint-cell';
      cell4.className = 'waypoint-cell';
      cell5.className = 'waypoint-cell';

      // Set value of cells
      cell1.innerHTML = '#';
      cell2.innerHTML = 'X';
      cell3.innerHTML = 'Y';
      cell4.innerHTML = 'Yaw';
      cell5.innerHTML = 'Remove';
    },
    // Add waypoint to html table
    addWaypointToList(wp) {
      // Get table
      const table = document.getElementById('waypoint-table');
      const lengthTable = table.rows.length;
      // Create remove button
      const removeBtn = document.createElement('button');
      removeBtn.id = `removeBtn-${lengthTable}`;
      removeBtn.className = 'removeBtn';
      removeBtn.onclick = function () { this.removeWaypointFromList(lengthTable); }.bind(this);
      // Create and set row
      const row = table.insertRow(lengthTable);
      row.className = 'table-row';
      // Add a cells in row
      const cell1 = row.insertCell(0);
      const cell2 = row.insertCell(1);
      const cell3 = row.insertCell(2);
      const cell4 = row.insertCell(3);
      const cell5 = row.insertCell(4);
      // Set the cells content
      cell1.innerHTML = lengthTable;
      cell2.innerHTML = wp.x.toFixed(1);
      cell3.innerHTML = wp.y.toFixed(1);
      cell4.innerHTML = wp.yaw.toFixed(1);
      cell5.appendChild(removeBtn);
    },
    // Remove waypoint from html table
    removeWaypointFromList(index) {
      this.waypointList.splice(index - 1, 1);
      const table = document.getElementById('waypoint-table');
      const lengthTable = table.rows.length;
      const lengthWaypoints = this.waypointList.length;
      for (var i = index; i < lengthTable; i++) {
        const row = table.deleteRow(index);
      }

      for (var i = index - 1; i < lengthWaypoints; i++) {
        const wp = this.waypointList[i];
        this.addWaypointToList(wp);
      }
    },
    // Get the right coordinate from canvas for the waypoint given
    getCanvasCoordinatesFromVideo(x, y) {
      const offsetAndScale = this.getVideoOffsetAndScale();

      return {
        x: x * offsetAndScale.scale + offsetAndScale.offsetX,
        y: y * offsetAndScale.scale + offsetAndScale.offsetY,
      };
    },
    // On mouse event
    onMouseDown(event) {
      if (event.button === 0) {
        const coord = this.getVideoCoordinatesFromEvent(event);
        if (this.isClickValid(coord)) {
          const wp = coord;
          wp.yaw = 0;
          this.addWaypoint(wp);
        }
      }
    },
    // Check is the click was inbound
    isClickValid(coord) {
      return coord.x >= 0 &&
        coord.x < this.videoElement.videoWidth &&
        coord.y >= 0 &&
        coord.y < this.videoElement.videoHeight;
    },
    // Send the waypoint list for patrol scheduling
    sendPatrol() {
      this.bus.$emit('send-patrol', JSON.stringify(this.waypointList));
    },
  },
};
</script>

<style>
#patrol-container{
  width: 100%;
  height: 100%;
}
.patrol-btn-container{
  height: 10%;
  padding: 2px;
  align-content: center
}
.title{
  width: 75%;
  height: 100%;
  font-size: 20pt;
  float: left;
  text-align: left;
}

/*
.list-container{
  width: 19%;
  height: 100%;
  position: relative;
  float: left;
  border: 1px solid #ddd;
}
*/
/*
.map-container{
  width: 80%;
  height: 100%;
  position: relative;
  float: right;
}
*/

</style>
