<template>
  <!-- Joystick widget -->
  <div
    id="joystick"
    class="h-100 w-100"
  >
    <!-- Joystick canvas -->
    <canvas
      ref="canvas"
      class="h-100 m-100"
      @mousedown="onMouseDown"
      @mouseup="onMouseUp"
      @mousemove="onMouseMove"
      @mouseout="onMouseOut"
    />
  </div>
</template>

<script>
/**
 * Vue SFC used as a widget that draws an joystick that the user
 * can use to send teleoperation control to the robot that it is
 * connected to. Takes 2 absolute values in props to set the max
 * value of a command and a bus to send the event (new joystick value).
 * This widget has the following dependencies : Bootstrap-Vue for styling.
 *
 * @module widget/Joystick
 * @vue-prop {Boolean} enable - Enable the sending of joystick data.
 * @vue-prop {Number} absoluteMaxX - Max x value of the joystick coordinate.
 * @vue-prop {Number} absoluteMaxY - Max y value of the joystick coordinate.
 * @vue-prop {Vue} bus - Vue bus use to emit event to other components.
 * @vue-event {Object} joystick-position-change - Emit joystick data to be sent to robot.
 * @vue-data {Number} x - Horizontal coordinate of the joystick.
 * @vue-data {Number} y - Vertical coordinate of the joystick.
 * @vue-data {Number} loopIntervalId - Refresh canvas loop (timer).
 * @vue-data {Number} positionChangeIntervalId - Joystick updating loop (timer).
 * @vue-data {HTMLCanvasElement} canvas - HTML element of the canvas.
 * @vue-data {HTMLCanvasElement} context - Canvas 2d context.
 * @vue-data {Number} radiusRatio - Size of the joystick in ratio of available space.
 * @vue-data {HTMLElement} joystickElement - HTML element of the joystick.
 * @vue-data {Boolean} isMouseDown - Keep a manual trace of click in canvas for drawing.
 * @vue-data {Number} canvasRefreshRate - Number of time to update canvas for 1 sec.
 * @vue-data {Number} operatorCommandInterval - Time in ms between the joystick postion update.
 */

/**
 * @author Edouard Legare <edouard.legare@usherbrooke.ca>,
 * @author Valerie Gauthier <valerie.gauthier4@usherbrooke.ca>,
 * @version 1.0.0
 */

import Vue from 'vue';

export default {
  name: 'joystick',
  props: {
    enable: {
      type: Boolean,
      required: true,
    },
    absoluteMaxX: {
      type: Number,
      required: true,
    },
    absoluteMaxY: {
      type: Number,
      required: true,
    },
    bus: {
      type: Vue,
      required: true,
    },
  },
  data() {
    return {
      x: null,
      y: null,
      loopIntervalId: null,
      positionChangeIntervalId: null,
      canvas: null,
      context: null,
      radiusRatio: 0.75,
      joystickElement: null,
      isMouseDown: false,
      canvasRefreshRate: 60.0, // Hz
      operatorCommandInterval: 100, // ms
    };
  },
  mounted() {
    this.joystickElement = document.getElementById('joystick');
    this.canvas = this.$refs.canvas;
    this.context = this.canvas.getContext('2d');
    this.init();
  },
  destroyed() {
    clearInterval(this.loopIntervalId);
    clearInterval(this.positionChangeIntervalId);
  },
  methods: {
    /**
     * Initilisation of component and timer.
     * @method
     */
    init() {
      // Timer refreshing the canvas
      this.loopIntervalId = setInterval(() => {
        this.setCanvasSize();
        this.findCenterCanvas();
        this.drawCanvas();
      }, 1000 / this.canvasRefreshRate);
      // Timer sending joystick position
      this.positionChangeIntervalId = setInterval(() => {
        this.emitJoystickPosition();
      }, this.operatorCommandInterval);
    },
    /**
     * Find the center of canvas
     * @method
     */
    findCenterCanvas() {
      if (this.x === null || this.y === null || !this.isMouseDown) {
        this.x = this.getCenterX();
        this.y = this.getCenterY();
      }
    },
    /**
     * Callback for the mouse down event.
     * @method
     * @param {HTMLElement} event - The html event given by the click.
     */
    onMouseDown(event) {
      if (event.button === 0) {
        this.updateJoystickPositionFromMouseEvent(event);
        this.isMouseDown = true;
      }
    },
    /**
     * Callback for the mouse up event.
     * @method
     * @param {HTMLElement} event - The html event given by the click.
     */
    onMouseUp(event) {
      if (event.button === 0) {
        this.x = this.getCenterX();
        this.y = this.getCenterY();
        this.isMouseDown = false;
        if (this.enable) {
          this.emitJoystickPosition();
        }
      }
    },
    /**
     * Callback for the mouse move event.
     * @method
     * @param {HTMLElement} event - The html event given by the click.
     */
    onMouseMove(event) {
      if (this.isMouseDown) {
        this.updateJoystickPositionFromMouseEvent(event);
      }
    },
    /**
     * Callback for the mouse out event.
     * @method
     * @param {HTMLElement} event - The html event given by the click.
     */
    onMouseOut() {
      this.x = this.getCenterX();
      this.y = this.getCenterY();
      this.isMouseDown = false;
      if (this.enable) {
        this.emitJoystickPosition();
      }
    },
    /**
     * Update the joystick position with the given event.
     * @method
     * @param {HTMLElement} event - The html event given by the click.
     */
    updateJoystickPositionFromMouseEvent(event) {
      const rect = this.canvas.getBoundingClientRect();
      this.x = event.clientX - rect.left;
      this.y = event.clientY - rect.top;

      const centerX = this.getCenterX();
      const centerY = this.getCenterY();
      const deltaX = this.x - centerX;
      const deltaY = this.y - centerY;
      const radius = Math.sqrt((deltaX * deltaX) + (deltaY * deltaY));
      const maxRadius = this.getCanvasRadius() - this.getJoystickRadius();

      if (radius > maxRadius) {
        const ratio = maxRadius / radius;
        this.x = (deltaX * ratio) + centerX;
        this.y = (deltaY * ratio) + centerY;
      }
    },
    /**
     * Calls the different methods to draw the canvas.
     * @method
     */
    drawCanvas() {
      this.context.clearRect(0, 0, this.canvas.width, this.canvas.height);
      this.drawBackground();
      this.drawJoystick();
    },
    /**
     * Draws the joystick movable circle.
     * @method
     */
    drawJoystick() {
      if (this.isMouseDown) {
        this.context.fillStyle = 'rgba(0, 0, 0, 0.75)';
      } else {
        this.context.fillStyle = '#000000';
      }

      this.context.beginPath();
      this.context.arc(this.x, this.y, this.getJoystickRadius(), 0, 2 * Math.PI);
      this.context.fill();
    },
    /**
     * Draws the joystick background.
     * @method
     */
    drawBackground() {
      const centerX = this.getCenterX();
      const centerY = this.getCenterY();

      const radius = this.getCanvasRadius();

      // Draw the background circle
      this.context.fillStyle = '#87CEEB';
      this.context.beginPath();
      this.context.arc(centerX, centerY, radius, 0, 2 * Math.PI);
      this.context.fill();


      const pointOffset = radius / 8;
      const halfPointOffset = pointOffset / 2;

      // draw center cross
      this.context.lineWidth = 2;
      this.context.strokeStyle = '#4682B4';
      this.context.beginPath();
      this.context.moveTo(centerX, centerY - pointOffset);
      this.context.lineTo(centerX, centerY + pointOffset);
      this.context.stroke();

      this.context.beginPath();
      this.context.moveTo(centerX - pointOffset, centerY);
      this.context.lineTo(centerX + pointOffset, centerY);
      this.context.stroke();


      // draw the up triangle
      const upTriangleStartY = centerY - ((3 * radius) / 4);

      this.context.fillStyle = '#4682B4';
      this.context.beginPath();
      this.context.moveTo(centerX, upTriangleStartY);
      this.context.lineTo(centerX - halfPointOffset, upTriangleStartY + pointOffset);
      this.context.lineTo(centerX + halfPointOffset, upTriangleStartY + pointOffset);
      this.context.fill();

      // draw the down triangle
      const downTriangleStartY = centerY + ((3 * radius) / 4);

      this.context.beginPath();
      this.context.moveTo(centerX, downTriangleStartY);
      this.context.lineTo(centerX - halfPointOffset, downTriangleStartY - pointOffset);
      this.context.lineTo(centerX + halfPointOffset, downTriangleStartY - pointOffset);
      this.context.fill();

      // draw the left triangle
      const leftTriangleStartX = centerX - ((3 * radius) / 4);

      this.context.beginPath();
      this.context.moveTo(leftTriangleStartX, centerY);
      this.context.lineTo(leftTriangleStartX + pointOffset, centerY - halfPointOffset);
      this.context.lineTo(leftTriangleStartX + pointOffset, centerY + halfPointOffset);
      this.context.fill();

      // draw the right triangle
      const rightTriangleStartX = centerX + ((3 * radius) / 4);

      this.context.beginPath();
      this.context.moveTo(rightTriangleStartX, centerY);
      this.context.lineTo(rightTriangleStartX - pointOffset, centerY - halfPointOffset);
      this.context.lineTo(rightTriangleStartX - pointOffset, centerY + halfPointOffset);
      this.context.fill();
    },
    /**
     * Set the size of the canvas.
     * @method
     */
    setCanvasSize() {
      this.canvas.width = this.joystickElement.clientWidth;
      this.canvas.height = this.joystickElement.clientHeight;
    },
    /**
     * Get the central horizontal value of canvas.
     * @method
     * @returns {Number} Canvas width divided by 2.
     */
    getCenterX() {
      return this.canvas.width / 2;
    },
    /**
     * Get the central vertical value of canvas.
     * @method
     * @returns {Number} Canvas height divided by 2.
     */
    getCenterY() {
      return this.canvas.height / 2;
    },
    /**
     * Get the center of the joystick's canvas.
     * @method
     * @returns {Number} Radius times the lowest value between height or width divided by 2.
     */
    getCanvasRadius() {
      return (this.radiusRatio * Math.min(this.canvas.width, this.canvas.height)) / 2;
    },
    /**
     * Get the joystick radius.
     * @method
     * @returns {Number} Canvas radius divided by 6.
     */
    getJoystickRadius() {
      return this.getCanvasRadius() / 6;
    },
    /**
     * Emit the joystick position to be sent to robot.
     * @method
     */
    emitJoystickPosition() {
      const event = {
        x: ((this.x - this.getCenterX()) * this.absoluteMaxX)
        / (this.getCanvasRadius() - this.getJoystickRadius()),
        y: ((this.y - this.getCenterY()) * this.absoluteMaxY)
        / (this.getCanvasRadius() - this.getJoystickRadius()),
      };
      if (this.enable) {
        this.bus.$emit('joystick-position-change', event);
      }
    },
  },
};
</script>

<style>
.inner-joystick-container{
  padding:10px;
  height: 100%;
  width: 100%;
}
</style>
