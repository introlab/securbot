<template>
  <!-- Patrol Map widget -->
  <div class="h-100 w-100 position-relative">
    <!-- Video Box -->
    <video-box
      :show="true"
      :video-id="patrolMapId"
      class="w-100 h-100 position-absolute"
      style="top:0;left:0;" />
    <!-- Canvas -->
    <canvas
      ref="canvas"
      class="w-100 h-100 position-absolute"
      style="top:0;left:0;z-index:10;"
      @mousedown="onMouseDown"
      @mousemove="onMouseMove"
      @mouseup="onMouseUp"
      @mouseout="onMouseOut" />
  </div>
</template>

<script>
/**
 * Vue SFC used as a widget to set waypoint on a map. This component
 * uses the VideoBox component to show the robot map (that is sent from
 * the robot in a video feed format) and put a canvas on top of it to
 * detect user clicks. When a click is detected the x and y position
 * is used to set a waypoint in the array given in props (push). The
 * canvas then read the array and draw waypoint on the map where the
 * user clicked previously.
 * This component have the following dependency :
 * VideoBox.vue Component and Bootstrap-Vue for styling.
 *
 *
 * @module widget/PatrolMap
 * @vue-prop {Object[]} waypointList - Lists the current waypoints
 * @vue-prop {String} patrolMapId - Identifies map source with exact name reference
 * @vue-data {Object} videoElement - Contains reference to video-id
 * @vue-data {Object} canvas - Contains reference to responsive overlay of map
 * @vue-data {Object} context - Sets canvas context
 * @vue-data {Number} CanvasRefreshRate - Sets the constant refresh rate of the displayed canvas
 * @vue-data {Object} loopIntervalId - Contains refresh rate and display parameter of the map
 * @vue-data {Boolean} enable - Enables or disables the display of the map and canvas
 */

/* Disabled comment documentation
 * Might use those eventually by forking jsdoc-vue-js so it can manage the author
 * and version tag correctly
 * @author Valerie Gauthier <valerie.gauthier@usherbrooke.ca>
 * @author Edouard Legare <edouard.legare@usherbrooke.ca>
 * @version 1.0.0
 */

import VideoBox from './VideoBox';

export default {
  name: 'patrol-map',
  components: {
    VideoBox,
  },
  props: {
    waypointList: {
      type: Array,
      default: () => [],
      required: true,
    },
    patrolMapId: {
      type: String,
      required: true,
    },
  },
  data() {
    return {
      videoElement: null,
      canvas: null,
      context: null,
      CanvasRefreshRate: 60.0, // Hz
      isMouseDown: false,
      loopIntervalId: null,
      enable: true,
    };
  },
  /**
   * Lifecycle Hook - mounted.
   * On component mounted, Get html elements and initialize.
   * @method
   * @listens mount(el)
   */
  mounted() {
    this.videoElement = document.getElementById(this.patrolMapId);
    this.canvas = this.$refs.canvas;
    this.context = this.canvas.getContext('2d');
    this.init();
  },
  /**
   * Lifecycle Hook - destroyed.
   * On component destroyed, clear refresh rate of canvas.
   * @method
   * @listens destroyed(el)
   */
  destroyed() {
    clearInterval(this.loopIntervalId);
  },
  methods: {
    /**
     * Initialisation of canvas refrash rate and call to canvas resizing functions.
     * @method
     */
    init() {
      this.loopIntervalId = setInterval(() => {
        if (this.enable) {
          this.adjustCanvasToVideo();
          this.drawCanvas();
        }
      }, 1000 / this.CanvasRefreshRate);
    },

    /**
     * Clears canvas and redraws the waypoints of the current patrol.
     * @method
     */
    drawCanvas() {
      this.context.clearRect(0, 0, this.canvas.width, this.canvas.height);
      this.drawWaypointList();
    },

    /**
     * Calls for the drawing the waypoints with corresponding arrows (indicating the yaws) of
     * each waypoint of the current waypoint list.
     * @method
     */
    drawWaypointList() {
      for (const [index, wp] of this.waypointList.entries()) {
        this.drawWaypoint(wp, index);
        this.drawYawArrow(wp);
      }
    },

    /**
     * Draws a waypoint on the canvas.
     * @method
     * @param {Object} wp - Waypoint
     * @param {Number} wp.x - Waypoint's x coordinate in pixel
     * @param {Number} wp.y - Waypoint's y coordinate in pixel
     * @param {Number} index - Index number of sent waypoint
     */
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

    /**
     * Draws arrow for the yaw of the waypoint on the canvas.
     * @method
     * @param {Object} wp - Waypoint
     * @param {Number} wp.x - Waypoint's x coordinate in pixel
     * @param {Number} wp.y - Waypoint's y coordinate in pixel
     * @param {Number} wp.yaw - Waypoint's yaw angle in radians
     */
    drawYawArrow(wp) {
      const arrowColor = '#00FF00';
      const coord = this.getCanvasCoordinatesFromVideo(wp.x, wp.y);

      const arrowLength = Math.min(this.canvas.width, this.canvas.height) / 15;
      const headLength = arrowLength / 4;
      const arrowEnd = {
        x: coord.x + arrowLength * Math.cos(wp.yaw),
        y: coord.y + arrowLength * Math.sin(wp.yaw),
      };
      const arrowTip1 = {
        x: arrowEnd.x - headLength * Math.cos(wp.yaw - Math.PI / 4),
        y: arrowEnd.y - headLength * Math.sin(wp.yaw - Math.PI / 4),
      };
      const arrowTip2 = {
        x: arrowEnd.x - headLength * Math.cos(wp.yaw + Math.PI / 4),
        y: arrowEnd.y - headLength * Math.sin(wp.yaw + Math.PI / 4),
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
     * Get position/coordinate of mouse event on video.
     * @method
     * @param {HTMLElement} - Event given by the click.
     * @returns {Object} X and Y coordinate in pixels of event (mouse position).
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
     * Sets canvas size (height and width) to match the size of the video.
     * @method
     */
    adjustCanvasToVideo() {
      this.canvas.width = this.videoElement.offsetWidth;
      this.canvas.height = this.videoElement.offsetHeight;
    },

    /**
     * Compute the offset and rescaling parameters of resized video from original content.
     * @method
     * @returns {Object} Offset in X, offset in Y and scaling ratios.
     */
    getVideoOffsetAndScale() {
      const videoRatio = this.videoElement.videoWidth / this.videoElement.videoHeight;

      let offsetX = 0;
      let offsetY = 0;
      let scale = 1;
      if ((this.videoElement.offsetHeight * videoRatio) > this.videoElement.offsetWidth) {
        scale = this.videoElement.osffsetWidth / this.videoElement.videoWidth;
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
     * Corrects the waypoint coordinate (x,y) from the offsets and scale parameters of video.
     * @method
     * @param {Number} x - Horizontal coordinate on canvas.
     * @param {Number} y - Vertical coordinate on canvas.
     * @returns {Object} - X and Y coordinate in pixels of event (mouse position).
     */
    getCanvasCoordinatesFromVideo(x, y) {
      const offsetAndScale = this.getVideoOffsetAndScale();

      return {
        x: (x * offsetAndScale.scale) + offsetAndScale.offsetX,
        y: (y * offsetAndScale.scale) + offsetAndScale.offsetY,
      };
    },

    /**
     * CallBack of mouse down event. Verify validity and initialise waypoint creation.
     * @method
     * @param {HTMLElement} event - Event element given by the click.
     */
    onMouseDown(event) {
      if (event.button === 0) {
        const coord = this.getVideoCoordinatesOfEvent(event);
        if (this.isClickValid(coord)) {
          const wp = coord;
          wp.yaw = 0;
          this.addWaypointCoord(wp);
        }
      }
      this.isMouseDown = true;
    },

    /**
     * Callback of mouse move event. Updates the yaw with mouse position.
     * @method
     * @param {HTMLElement} event - Event given by mouse move.
     */
    onMouseMove(event) {
      if (this.isMouseDown) {
        console.log('MouseMoved');
        const wp = this.waypointList[this.waypointList.length - 1];
        const mousePosition = this.getVideoCoordinatesOfEvent(event);
        wp.yaw = Math.atan2(mousePosition.y - wp.y,
          mousePosition.x - wp.x);
        this.updateWaypoint(wp);
      }
    },

    /**
     * Callback of mouse up event, finalize the waypoint creation process.
     * @method
     * @param {HTMLElement} event - Event element given by the click.
     */
    onMouseUp(event) {
      if (this.isMouseDown) {
        // Write waypoint to list of waypoints
        console.log('MouseUP');
        const date = new Date();
        const wp = this.waypointList[this.waypointList.length - 1];
        const coord = this.getVideoCoordinatesOfEvent(event);
        wp.yaw = Math.atan2(coord.y - wp.y,
          coord.x - wp.x);
        wp.dateTime = date.getTime();
        this.updateWaypoint(wp);
        this.isMouseDown = false;
      }
    },

    /**
     * Callback of mouse out event, terminate the waypoint creation.
     * @method
     * @param {HTMLElement} event - Event element given by the click.
     */
    onMouseOut(event) {
      if (this.isMouseDown) {
        console.log('MouseOut');
        this.waypointList.pop();
        this.isMouseDown = false;
      }
    },

    /**
     * Adds a waypoint to the list of current waypoints.
     * @method
     * @param {Object} wp - Waypoint.
     */
    addWaypointCoord(wp) {
      this.waypointList.push(wp);
    },

    /**
     * Updates last waypoint of the waypoint list.
     * @method
     * @param {Object} wp - Waypoint.
     */
    updateWaypoint(wp) {
      this.waypointList.pop();
      this.waypointList.push(wp);
    },

    /**
     * Check to see if element is in bound.
     * @method
     * @param {Object} coord - Coordinates of an element.
     * @param {Number} coord.x - X coordinate of element.
     * @param {Number} coord.y - Y coordinate of element.
     * @returns {boolean} true if coordinates of the element are in bounds of the video.
     */
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
