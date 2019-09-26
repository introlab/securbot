<template>
  <!-- Patrol Map widget -->
  <div class="h-100 w-100 position-relative">
    <!-- Video Box -->
    <video-box
      :show="true"
      :video-id="patrolId"
      class="w-100 h-100 position-absolute"
      style="top:0;left:0;"
    />
    <!-- Canvas -->
    <canvas
      ref="canvas"
      class="w-100 h-100 position-absolute"
      style="top:0;left:0;z-index:10;"
      @mousedown="onMouseDown"
      @mousemove="onMouseMove"
      @mouseup="onMouseUp"
      @mouseout="onMouseOut"
    />
  </div>
</template>

<script>
/**
 * @author Valerie Gauthier <valerie.gauthier@usherbrooke.ca>
 * @author Edouard Legare <edouard.legare@usherbrooke.ca>
 * @version 1.0.0
 */

import { mapState } from 'vuex';
import VideoBox from './VideoBox';

export default {
  name: 'patrol-map',
  components: {
    VideoBox,
  },
  data() {
    return {
      canvas: null,
      context: null,
      CanvasRefreshRate: 30.0, // Hz
      isMouseDown: false,
      loopIntervalId: null,
      enable: true,
    };
  },
  computed: mapState({
    mapStream: state => state.client.mapStream,
    waypointList: state => state.patrol.waypointList,
    patrolId: state => state.htmlElement.patrolId,
    videoElement: state => state.htmlElement.patrol,
  }),
  mounted() {
    this.canvas = this.$refs.canvas;
    this.context = this.canvas.getContext('2d');
    this.init();
  },
  destroyed() {
    clearInterval(this.loopIntervalId);
  },
  methods: {
    init() {
      this.loopIntervalId = setInterval(() => {
        if (this.videoElement) {
          this.adjustCanvasToVideo();
          this.drawCanvas();
        }
      }, 1000 / this.CanvasRefreshRate);
    },
    drawCanvas() {
      this.context.clearRect(0, 0, this.canvas.width, this.canvas.height);
      this.drawWaypointList();
    },
    drawWaypointList() {
      for (const [index, wp] of this.waypointList.entries()) {
        this.drawWaypoint(wp, index);
        this.drawYawArrow(wp);
      }
    },
    drawWaypoint(wp, index) {
      const wpColor = '#00FF00';
      const coord = this.getCanvasCoordinatesFromVideo(wp.x, wp.y);

      const wpRadius = 7;
      this.context.beginPath();
      this.context.arc(coord.x, coord.y, wpRadius, 0, 2 * Math.PI);
      this.context.fillStyle = wpColor;
      this.context.fill();

      this.context.font = '20px serif';
      this.context.fillStyle = '#000000';
      this.context.fillText(index + 1, coord.x + 8, coord.y + 8, 25);
    },
    drawYawArrow(wp) {
      const arrowColor = '#00FF00';
      const coord = this.getCanvasCoordinatesFromVideo(wp.x, wp.y);

      const arrowLength = Math.min(this.canvas.width, this.canvas.height) / 15;
      const headLength = arrowLength / 4;
      const radYaw = -wp.yaw / 180 * Math.PI;
      const arrowEnd = {
        x: coord.x + arrowLength * Math.cos(radYaw),
        y: coord.y + arrowLength * Math.sin(radYaw),
      };
      const arrowTip1 = {
        x: arrowEnd.x - headLength * Math.cos(radYaw - Math.PI / 4),
        y: arrowEnd.y - headLength * Math.sin(radYaw - Math.PI / 4),
      };
      const arrowTip2 = {
        x: arrowEnd.x - headLength * Math.cos(radYaw + Math.PI / 4),
        y: arrowEnd.y - headLength * Math.sin(radYaw + Math.PI / 4),
      };

      this.context.lineCap = 'round';
      this.context.lineWidth = Math.max(1, arrowLength / 10);
      this.context.strokeStyle = arrowColor;

      this.context.beginPath();
      this.context.moveTo(coord.x, coord.y);
      this.context.lineTo(arrowEnd.x, arrowEnd.y);
      this.context.stroke();

      this.context.beginPath();
      this.context.moveTo(arrowEnd.x, arrowEnd.y);
      this.context.lineTo(arrowTip1.x, arrowTip1.y);
      this.context.stroke();

      this.context.beginPath();
      this.context.moveTo(arrowEnd.x, arrowEnd.y);
      this.context.lineTo(arrowTip2.x, arrowTip2.y);
      this.context.stroke();
    },
    getVideoCoordinatesOfEvent(event) {
      const offsetAndScale = this.getVideoOffsetAndScale();
      const rect = this.videoElement.getBoundingClientRect();
      const x = (event.clientX - rect.left - offsetAndScale.offsetX) / offsetAndScale.scale;
      const y = (event.clientY - rect.top - offsetAndScale.offsetY) / offsetAndScale.scale;
      return {
        x,
        y,
      };
    },
    adjustCanvasToVideo() {
      this.canvas.width = this.videoElement.offsetWidth;
      this.canvas.height = this.videoElement.offsetHeight;
    },
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
    getCanvasCoordinatesFromVideo(x, y) {
      const offsetAndScale = this.getVideoOffsetAndScale();

      return {
        x: (x * offsetAndScale.scale) + offsetAndScale.offsetX,
        y: (y * offsetAndScale.scale) + offsetAndScale.offsetY,
      };
    },
    onMouseDown(event) {
      if (event.button === 0 && this.mapStream) {
        console.log('onMouseDown');
        const coord = this.getVideoCoordinatesOfEvent(event);
        if (this.isClickValid(coord)) {
          const wp = coord;
          wp.yaw = 0;
          this.addWaypointCoord(wp);
          this.isMouseDown = true;
        }
      }
    },
    onMouseMove(event) {
      if (this.isMouseDown) {
        console.log('MouseMoved');
        const wp = this.waypointList[this.waypointList.length - 1];
        const mousePosition = this.getVideoCoordinatesOfEvent(event);
        wp.yaw = -Math.atan2(mousePosition.y - wp.y, mousePosition.x - wp.x) * 180 / Math.PI;
        this.updateWaypoint(wp);
      }
    },
    onMouseUp(event) {
      if (this.isMouseDown) {
        // Write waypoint to list of waypoints
        console.log('MouseUP');
        const date = new Date();
        const wp = this.waypointList[this.waypointList.length - 1];
        const coord = this.getVideoCoordinatesOfEvent(event);

        wp.yaw = -Math.atan2(coord.y - wp.y, coord.x - wp.x) * 180 / Math.PI;
        wp.dateTime = date.getTime();
        this.updateWaypoint(wp);
        this.isMouseDown = false;
      }
    },
    onMouseOut() {
      if (this.isMouseDown) {
        console.log('MouseOut');
        this.$store.commit('removeWaypoint', this.waypointList.length - 1);
        this.isMouseDown = false;
      }
    },
    addWaypointCoord(wp) {
      this.$store.commit('addWaypoint', { wp });
    },
    updateWaypoint(wp) {
      const update = {
        wp,
        index: this.waypointList.length - 1,
      };
      this.$store.commit('updateWaypoint', update);
    },
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
