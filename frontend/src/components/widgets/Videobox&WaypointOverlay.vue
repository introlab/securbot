<template>
  <div class="h-100 w-100 position-relative">
    <video-box
      :show="true"
      :video-id="videoId"
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
import VideoBox from './VideoBox';

export default {
  name: 'waypoint-overlay',
  components: {
    VideoBox,
  },
  props: {
    componentName: {
      type: String,
      required: true,
    },
  },
  data() {
    return {
      waypoint: { x: 0, y: 0, yaw: 0 },
      videoElement: null,
      canvas: null,
      context: null,
      loopIntervalId: null,
      CanvasRefreshRate: 60.0, // Hz
      isMouseDown: false,
    };
  },
  computed: {
    videoId() {
      return `${this.componentName}-videobox`;
    },
  },
  mounted() {
    this.videoElement = document.getElementById(this.videoId);
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
        this.adjustCanvasToVideo();
        this.drawCanvas();
      }, 1000 / this.CanvasRefreshRate);
    },
    drawCanvas() {
      this.context.clearRect(0, 0, this.canvas.width, this.canvas.height);
      this.draw();
    },
    draw() {
      this.drawWaypoint(this.waypoint);
      this.drawYawArrow(this.waypoint);
    },
    drawWaypoint(wp) {
      const wpColor = '#00FF00';
      const coord = this.getCanvasCoordinatesFromVideo(wp.x, wp.y);

      const wpRadius = 7;
      this.context.beginPath();
      this.context.arc(coord.x, coord.y, wpRadius, 0, 2 * Math.PI);
      this.context.fillStyle = wpColor;
      this.context.fill();
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
      if (event.button === 0) {
        console.log('onMouseDown');
        const coord = this.getVideoCoordinatesOfEvent(event);
        if (this.isClickValid(coord)) {
          const wp = coord;
          wp.yaw = 0;
          this.waypoint = wp;
          this.isMouseDown = true;
        }
      }
    },
    onMouseMove(event) {
      if (this.isMouseDown) {
        console.log('MouseMoved');
        const mousePosition = this.getVideoCoordinatesOfEvent(event);
        // eslint-disable-next-line max-len
        this.waypoint.yaw = -Math.atan2(mousePosition.y - this.waypoint.y, mousePosition.x - this.waypoint.x) * 180 / Math.PI;
      }
    },
    onMouseUp(event) {
      if (this.isMouseDown) {
        // Write waypoint to list of waypoints
        console.log('MouseUP');
        // const date = new Date();
        const coord = this.getVideoCoordinatesOfEvent(event);

        // eslint-disable-next-line max-len
        this.waypoint.yaw = -Math.atan2(coord.y - this.waypoint.y, coord.x - this.waypoint.x) * 180 / Math.PI;
        this.isMouseDown = false;
      }
    },
    onMouseOut() {
      if (this.isMouseDown) {
        console.log('MouseOut');
        this.waypoint = { x: 0, y: 0, yaw: 0 };
        this.isMouseDown = false;
      }
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

<style scoped>

</style>
