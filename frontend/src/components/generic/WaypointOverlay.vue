<template>
  <!-- Canvas -->
  <div>
    <canvas
      ref="canvas"
      class="w-100 h-100 position-absolute"
      style="top:0;left:0;z-index:10;"
      @mousedown="onMouseDown"
      @mousemove="onMouseMove"
      @mouseup="onMouseUp"
      @mouseout="onMouseOut"
      @mouseover="showOverlay = true"
    />
  </div>
</template>

<script>
/**
 * An overlay component for the videobox component that allows an operator to click the video to
 * add a waypoint while also drawing all the waypoints from a list given as a prop.
 *
 * NOTE: This component do not add waypoints into the prop list to assure compatibility with Vuex's
 * state behavior. Instead, the component draws the prop list as waypoint plus a local waypoint
 * that gets its coordinates from an operator click (otherwise is it not drawn). If the operator
 * click was deemed valid, the local waypoint is then emitted through a component event and is the
 * parent component responsability to add to the waypoint prop list.
 *
 * @displayName Waypoint Overlay
 * @since 0.2.2
 * @version 1.0.0
 */
export default {
  name: 'waypoint-overlay',
  props: {
    isActive: {
      type: Boolean,
      default: true,
    },
    isClickable: {
      type: Boolean,
      default: true,
    },
    show: {
      type: Boolean,
      default: true,
    },
    list: {
      type: Array,
      required: true,
    },
    nbOfWaypoint: {
      type: Number,
      default: -1,
    },
    videoElement: {
      type: HTMLVideoElement,
      required: true,
    },
    refreshRate: {
      type: Number,
      default: 30,
    },
  },
  data() {
    return {
      canvas: null,
      context: null,
      currentWaypoint: {
        x: -1,
        y: -1,
        yaw: 0,
      },
      isMouseDown: false,
      loopIntervalId: null,
      showOverlay: false,
    };
  },
  mounted() {
    this.$nextTick(() => {
      this.canvas = this.$refs.canvas;
      this.context = this.canvas.getContext('2d');
      this.init();
    });
  },
  destroyed() {
    clearInterval(this.loopIntervalId);
  },
  methods: {
    /**
     * Initializes the Patrol map overlay.
     *
     * @public
     */
    init() {
      this.loopIntervalId = setInterval(() => {
        if (this.videoElement && this.isActive) {
          this.adjustCanvasToVideo();
          this.drawCanvas();
        }
      }, 1000 / this.refreshRate);
      if (this.videoElement) {
        this.drawWaypointList();
      }
    },
    /**
     * Draws the canvas.
     *
     * @public
     */
    drawCanvas() {
      this.context.clearRect(0, 0, this.canvas.width, this.canvas.height);
      this.drawWaypointList();
    },
    /**
     * Draws the points and arrows of each waypoint in the list.
     *
     * @public
     */
    drawWaypointList() {
      if (this.nbOfWaypoint >= 0) {
        for (let i = 0; i < this.nbOfWaypoint && i < this.list.length; i++) {
          this.drawWaypoint(this.list[i], -1);
          this.drawYawArrow(this.list[i]);
        }
      } else {
        for (const [index, wp] of this.list.entries()) {
          this.drawWaypoint(wp, index);
          this.drawYawArrow(wp);
        }
      }
      if (this.isMouseDown) {
        this.drawWaypoint(this.currentWaypoint, (this.nbOfWaypoint >= 0 ? -1 : this.list.length));
        this.drawYawArrow(this.currentWaypoint);
      }
    },
    /**
     * Draws a the point/dot.
     *
     * @param {Waypoint} wp The waypoint object.
     * @param {Number} index The index of the waypoint in the list (Is written next to the dot).
     * @public
     */
    drawWaypoint(wp, index) {
      const wpColor = '#00FF00';
      const coord = this.getCanvasCoordinatesFromVideo(wp.x, wp.y);

      const wpRadius = 7;
      this.context.beginPath();
      this.context.arc(coord.x, coord.y, wpRadius, 0, 2 * Math.PI);
      this.context.fillStyle = wpColor;
      this.context.fill();

      if (index >= 0) {
        this.context.font = '20px serif';
        this.context.fillStyle = '#000000';
        this.context.fillText(index + 1, coord.x + 8, coord.y + 8, 25);
      }
    },
    /**
     * Draws the arrows.
     *
     * @param {Waypoint} wp The waypoint object.
     * @public
     */
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
    /**
     * Gets the coordinates of the operator click.
     *
     * @param {Event} event The mouse event to extract data from.
     * @public
     */
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
    /**
     * Adjusts the canvas to the video size.
     *
     * @public
     */
    adjustCanvasToVideo() {
      this.canvas.width = this.videoElement.offsetWidth;
      this.canvas.height = this.videoElement.offsetHeight;
    },
    /**
     * Gets the offset and scale difference between the video and the canvas.
     *
     * @public
     */
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
    /**
     * Converts video/map coordinates to canvas coordinates.
     *
     * @param {Number} x The value of the x coordinate.
     * @param {Number} y The value of the y coordinate.
     * @public
     */
    getCanvasCoordinatesFromVideo(x, y) {
      const offsetAndScale = this.getVideoOffsetAndScale();

      return {
        x: (x * offsetAndScale.scale) + offsetAndScale.offsetX,
        y: (y * offsetAndScale.scale) + offsetAndScale.offsetY,
      };
    },
    /**
     * Gets called when the operator clicks the canvas.
     *
     * @param {Event} event The click event.
     * @public
     */
    onMouseDown(event) {
      if (event.button === 0 && this.isActive && this.isClickable) {
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
    /**
     * Gets called when the operator moves its mouse after clicking..
     *
     * @param {Event} event The move event.
     * @public
     */
    onMouseMove(event) {
      if (this.isMouseDown) {
        console.log('MouseMoved');
        const mousePosition = this.getVideoCoordinatesOfEvent(event);
        this.currentWaypoint.yaw = -Math.atan2(
          mousePosition.y - this.currentWaypoint.y,
          mousePosition.x - this.currentWaypoint.x,
        ) * 180 / Math.PI;
      }
    },
    /**
     * Gets called when the operator release the mouse button.
     *
     * @param {Event} event The release event.
     * @public
     */
    onMouseUp(event) {
      if (this.isMouseDown) {
        console.log('MouseUP');
        const mousePosition = this.getVideoCoordinatesOfEvent(event);

        this.currentWaypoint.yaw = -Math.atan2(
          mousePosition.y - this.currentWaypoint.y,
          mousePosition.x - this.currentWaypoint.x,
        ) * 180 / Math.PI;
        this.emitWaypoint();
        this.isMouseDown = false;

        this.currentWaypoint = {
          x: -1,
          y: -1,
          yaw: 0,
        };

        this.drawCanvas();
      }
    },
    /**
     * Gets called when the operator mouse exit the canvas.
     *
     * @public
     */
    onMouseOut() {
      if (this.isMouseDown) {
        console.log('MouseOut');
        this.currentWaypoint = {
          x: -1,
          y: -1,
          yaw: 0,
        };
        this.isMouseDown = false;
      }
    },
    /**
     * Adds a waypoint to the list.
     *
     * @param {Waypoint} wp The waypoint object.
     * @public
     */
    addWaypointCoord(wp) {
      Object.assign(this.currentWaypoint, wp);
    },
    /**
     * Updates the current waypoint.
     *
     * @param {Waypoint} wp The waypoint object.
     * @public
     */
    emitWaypoint() {
      /**
       * The new waypoint event.
       *
       * @type {Object} The valid internal waypoint.
       */
      this.$emit('newWaypoint', this.currentWaypoint);
    },
    /**
     * Verifies if the operator click was valid.
     *
     * @param {Object} coord An object containing the x and y coordinate to validate.
     * @public
     */
    isClickValid(coord) {
      return coord.x >= 0
        && coord.x < this.videoElement.videoWidth
        && coord.y >= 0
        && coord.y < this.videoElement.videoHeight;
    },
    // showOverlay() {
    //   // Create gradient
    //   const grd = this.context.createRadialGradient(75, 50, 5, 90, 60, 100);
    //   grd.addColorStop(0, '#00000000');
    //   grd.addColorStop(1, 'gray');

    //   // Fill with gradient
    //   this.context.fillStyle = grd;
    //   this.context.fillRect(10, 10, 150, 80);
    // },
  },
};
</script>

<style scoped>
</style>
