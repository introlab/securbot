<template>
  <div class="h-100 w-100 position-relative">
    <video-box
      :show="true"
      :video-id="patrolMapId"
      class="w-100 h-100 position-absolute"
      style="top:0;left:0;" />
    <canvas
      ref="canvas"
      class="w-100 h-100 position-absolute"
      style="top:0;left:0;z-index:10;"
      @mousedown="onMouseDown" />
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

import VideoBox from './VideoBox';

export default {
  name: 'patrol-map',
  components: {
    VideoBox,
  },
  props: ['waypointList', 'patrolMapId'],
  data() {
    return {
      videoElement: null,
      canvas: null,
      context: null,
      CanvasRefreshRate: 60.0, // Hz
      loopIntervalId: null,
      enable: true,
    };
  },
  // On component mounted, Get html elements and initialize
  mounted() {
    this.videoElement = document.getElementById(this.patrolMapId);
    this.canvas = this.$refs.canvas;
    this.context = this.canvas.getContext('2d');
    this.init();
  },
  // On component destroyed, clear refresh rate of canvas
  destroyed() {
    clearInterval(this.loopIntervalId);
  },
  methods: {
    // Initialisation of canvas refrash rate
    init() {
      this.loopIntervalId = setInterval(() => {
        if (this.enable) {
          this.adjustCanvasToVideo();
          this.drawCanvas();
        }
      }, 1000 / this.CanvasRefreshRate);
    },
    // Add waypoint to the list
    addWaypoint(wp) {
      this.waypointList.push(wp);
    },
    // Clean canvas and redraw the waypoints
    drawCanvas() {
      this.context.clearRect(0, 0, this.canvas.width, this.canvas.height);
      this.drawWaypoints();
    },
    // Draw waypoints on canvas
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
    // Get position/coordinate of video on click
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
        offsetY = (this.videoElement.offsetHeight - (this.videoElement.videoHeight * scale)) / 2;
      } else {
        scale = this.videoElement.offsetHeight / this.videoElement.videoHeight;
        offsetX = (this.videoElement.offsetWidth - (this.videoElement.videoWidth * scale)) / 2;
      }
      return {
        offsetX,
        offsetY,
        scale,
      };
    },
    // Make correction to the coordinates from canvas for the waypoint given
    getCanvasCoordinatesFromVideo(x, y) {
      const offsetAndScale = this.getVideoOffsetAndScale();

      return {
        x: (x * offsetAndScale.scale) + offsetAndScale.offsetX,
        y: (y * offsetAndScale.scale) + offsetAndScale.offsetY,
      };
    },
    // On mouse down event, verify validity of click
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
      return coord.x >= 0
        && coord.x < this.videoElement.videoWidth
        && coord.y >= 0
        && coord.y < this.videoElement.videoHeight;
    },
  },
};
</script>

<style>
</style>
