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
      @touchstart="onMouseDown"
      @mouseup="onMouseUp"
      @touchend="onMouseUp"
      @mousemove="onMouseMove"
      @touchmove="onMouseMove"
      @mouseout="onMouseOut"
    />
  </div>
</template>

<script>
/**
 * A joystick widget that be used by an operator to send data to the robot and control it.
 *
 * Authors:
 *
 *    - Valerie Gauthier - <valerie.gauthier@usherbrooke.ca>
 *    - Edouard Legare - <edouard.legare@usherbrooke.ca>
 *
 * @since 0.1.0
 * @version 1.0.0
 * @displayName Joystick
 */
export default {
  name: 'joystick',
  props: {
    /**
     * The state of the joystick. On true, joystick starts to send data to robot.
     */
    enable: {
      type: Boolean,
      required: true,
    },
    /**
     * The max X value of the joystick.
     */
    absoluteMaxX: {
      type: Number,
      required: true,
    },
    /**
     * The max Y value of the joystick.
     */
    absoluteMaxY: {
      type: Number,
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
     * Initialzes the joystick.
     *
     * @public
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
        if (this.enable) {
          this.emitJoystickPosition();
        }
      }, this.operatorCommandInterval);
    },
    /**
     * Finds the center of the canvas.
     *
     * @public
     */
    findCenterCanvas() {
      if (this.x === null || this.y === null || !this.isMouseDown) {
        this.x = this.getCenterX();
        this.y = this.getCenterY();
      }
    },
    /**
     * Called when the operator clicks the joystick.
     *
     * @param {Event} event The mouse click event
     * @public
     */
    onMouseDown(event) {
      if (event.button === 0) {
        this.updateJoystickPositionFromMouseEvent(event);
        this.isMouseDown = true;
      } else if (event.type === 'touchstart') {
        this.updateJoystickPositionFromMouseEvent(event.touches[0]);
        this.isMouseDown = true;
      }
    },
    /**
     * Called when the operator stop pressing the mouse button on the joystick.
     *
     * @param {Event} event The mouse click event
     * @public
     */
    onMouseUp(event) {
      if (event.button === 0 || event.type === 'touchend') {
        this.x = this.getCenterX();
        this.y = this.getCenterY();
        this.isMouseDown = false;
        if (this.enable) {
          this.emitJoystickPosition();
        }
      }
    },
    /**
     * Called when the operator moves the joystick.
     *
     * @param {Event} event The mouse click event
     * @public
     */
    onMouseMove(event) {
      if (this.isMouseDown) {
        if (event.clientX) {
          this.updateJoystickPositionFromMouseEvent(event);
        } else {
          this.updateJoystickPositionFromMouseEvent(event.touches[0]);
        }
      }
    },
    /**
     * Called when the operator moves its mouse out of the joystick zone.
     *
     * @public
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
     * Updates the joystick position.
     *
     * @param {Event} event The mouse event to update the joystick to.
     * @public
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
     * Draws the canvas.
     *
     * @public
     */
    drawCanvas() {
      this.context.clearRect(0, 0, this.canvas.width, this.canvas.height);
      this.drawBackground();
      this.drawJoystick();
    },
    /**
     * Draws the joystick.
     *
     * @public
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
     * Draws the background.
     *
     * @public
     */
    drawBackground() {
      const centerX = this.getCenterX();
      const centerY = this.getCenterY();

      const radius = this.getCanvasRadius();

      // Draw the background circle
      this.context.fillStyle = '#00A759';
      this.context.beginPath();
      this.context.arc(centerX, centerY, radius, 0, 2 * Math.PI);
      this.context.fill();


      const pointOffset = radius / 8;
      const halfPointOffset = pointOffset / 2;

      // draw center cross
      this.context.lineWidth = 2;
      this.context.strokeStyle = '#222222';
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

      this.context.fillStyle = '#222222';
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
     * Resizes the canvas.
     *
     * @public
     */
    setCanvasSize() {
      this.canvas.width = this.joystickElement.clientWidth;
      this.canvas.height = this.joystickElement.clientHeight;
    },
    /**
     * Gets the middle of the canvas, X axis.
     *
     * @public
     */
    getCenterX() {
      return this.canvas.width / 2;
    },
    /**
     * Gets the middle of the canvas, Y axis.
     *
     * @public
     */
    getCenterY() {
      return this.canvas.height / 2;
    },
    /**
     * Gets the canavs radius.
     *
     * @public
     */
    getCanvasRadius() {
      return (this.radiusRatio * Math.min(this.canvas.width, this.canvas.height)) / 2;
    },
    /**
     * Gets the joystick radius.
     *
     * @public
     */
    getJoystickRadius() {
      return this.getCanvasRadius() / 6;
    },
    /**
     * Sends the joystick position to the robot.
     *
     * @public
     */
    emitJoystickPosition() {
      const event = {
        x: ((this.x - this.getCenterX()) * this.absoluteMaxX)
        / (this.getCanvasRadius() - this.getJoystickRadius()),
        y: ((this.y - this.getCenterY()) * this.absoluteMaxY)
        / (this.getCanvasRadius() - this.getJoystickRadius()),
        enabled: this.enable,
      };
      if (this.enable) {
        this.$emit('onPosition', event);
      }
    },
  },
};
</script>

<style>
.inner-joystick-container{
  padding: 10px;
  height: 100%;
  width: 100%;
}
</style>
