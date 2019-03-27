<template>
  <div
    id="joystick"
    class="h-100 w-100 p-2">
    <canvas
      ref="canvas"
      class="h-100 m-100"
      @mousedown="onMouseDown"
      @mouseup="onMouseUp"
      @mousemove="onMouseMove"
      @mouseout="onMouseOut"/>
  </div>
</template>

<script>
/*
* Author(s):  Edouard Legare <edouard.legare@usherbrooke.ca>,
*             Valerie Gauthier <valerie.gauthier4@usherbrooke.ca>,
* File :  Joystick.vue
* Desc :  Vue SFC used as a widget that draws an joystick that the user
*         can use to send teleoperation control to the robot that it is
*         connected to. Takes 2 absolute values in props to set the max
*         value of a command and a bus to send the event (new command).
*         The width and height are not really used anymore in term of
*         props since the joystick size is now dynamic.
*
* Dependencies :
*       -Bootstrap-Vue
*
* Note :  The original file was given by [redacted] and modify afterward.
*         Aka, we are not the original author(s), but were given the right
*         to use and modify the file.
*
*/


export default {
  name: 'joystick',
  props: ['width', 'height', 'absoluteMaxX', 'absoluteMaxY', 'bus'],
  data() {
    return {
      x: null,
      y: null,
      loopIntervalId: null,
      positionChangeIntervalId: null,
      canvas: null,
      context: null,
      enable: false,
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
    init() {
      this.loopIntervalId = setInterval(() => {
        this.setCanvasSize();
        this.findCenterCanvas();
        this.drawCanvas();
      }, 1000 / this.canvasRefreshRate);
      if (this.enable) {
        this.positionChangeIntervalId = setInterval(() => {
          this.emitJoystickPosition();
        }, this.operatorCommandInterval);
      }
    },
    findCenterCanvas() {
      if (this.x === null || this.y === null || !this.isMouseDown) {
        this.x = this.getCenterX();
        this.y = this.getCenterY();
      }
    },
    onMouseDown(event) {
      if (event.button === 0) {
        this.updateJoystickPositionFromMouseEvent(event);
        this.isMouseDown = true;
      }
    },
    onMouseUp(event) {
      if (event.button === 0) {
        this.x = this.getCenterX();
        this.y = this.getCenterY();
        this.isMouseDown = false;
        this.emitJoystickPosition();
      }
    },
    onMouseMove(event) {
      if (this.isMouseDown) {
        this.updateJoystickPositionFromMouseEvent(event);
      }
    },
    onMouseOut(event) {
      this.x = this.getCenterX();
      this.y = this.getCenterY();
      this.isMouseDown = false;
      this.emitJoystickPosition();
    },
    updateJoystickPositionFromMouseEvent(event) {
      const rect = this.canvas.getBoundingClientRect();
      this.x = event.clientX - rect.left;
      this.y = event.clientY - rect.top;

      const centerX = this.getCenterX();
      const centerY = this.getCenterY();
      const deltaX = this.x - centerX;
      const deltaY = this.y - centerY;
      const radius = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
      const maxRadius = this.getCanvasRadius() - this.getJoystickRadius();

      if (radius > maxRadius) {
        const ratio = maxRadius / radius;
        this.x = deltaX * ratio + centerX;
        this.y = deltaY * ratio + centerY;
      }
      this.emitJoystickPosition();
    },
    drawCanvas() {
      this.context.clearRect(0, 0, this.canvas.width, this.canvas.height);
      this.drawBackground();
      this.drawJoystick();
    },
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
      const upTriangleStartY = centerY - 3 * radius / 4;

      this.context.fillStyle = '#4682B4';
      this.context.beginPath();
      this.context.moveTo(centerX, upTriangleStartY);
      this.context.lineTo(centerX - halfPointOffset, upTriangleStartY + pointOffset);
      this.context.lineTo(centerX + halfPointOffset, upTriangleStartY + pointOffset);
      this.context.fill();

      // draw the down triangle
      const downTriangleStartY = centerY + 3 * radius / 4;

      this.context.beginPath();
      this.context.moveTo(centerX, downTriangleStartY);
      this.context.lineTo(centerX - halfPointOffset, downTriangleStartY - pointOffset);
      this.context.lineTo(centerX + halfPointOffset, downTriangleStartY - pointOffset);
      this.context.fill();

      // draw the left triangle
      const leftTriangleStartX = centerX - 3 * radius / 4;

      this.context.beginPath();
      this.context.moveTo(leftTriangleStartX, centerY);
      this.context.lineTo(leftTriangleStartX + pointOffset, centerY - halfPointOffset);
      this.context.lineTo(leftTriangleStartX + pointOffset, centerY + halfPointOffset);
      this.context.fill();

      // draw the right triangle
      const rightTriangleStartX = centerX + 3 * radius / 4;

      this.context.beginPath();
      this.context.moveTo(rightTriangleStartX, centerY);
      this.context.lineTo(rightTriangleStartX - pointOffset, centerY - halfPointOffset);
      this.context.lineTo(rightTriangleStartX - pointOffset, centerY + halfPointOffset);
      this.context.fill();
    },
    setCanvasSize() {
      this.canvas.width = this.joystickElement.clientWidth;
      this.canvas.height = this.joystickElement.clientHeight;
    },
    getCenterX() {
      return this.canvas.width / 2;
    },
    getCenterY() {
      return this.canvas.height / 2;
    },
    getCanvasRadius() {
      return this.radiusRatio * Math.min(this.canvas.width, this.canvas.height) / 2;
    },
    getJoystickRadius() {
      return this.getCanvasRadius() / 4;
    },
    emitJoystickPosition() {
      const event = {
        x: (this.x - this.getCenterX()) * this.absoluteMaxX / (this.getCanvasRadius() - this.getJoystickRadius()),
        y: (this.y - this.getCenterY()) * this.absoluteMaxY / (this.getCanvasRadius() - this.getJoystickRadius()),
      };
      this.bus.$emit('joystick-position-change', event);
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
