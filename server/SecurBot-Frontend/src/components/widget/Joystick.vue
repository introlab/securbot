<template>
  <div id="joystick" class="inner-joystick-container">
    <canvas ref="canvas" class="full"
    @mousedown="onMouseDown" @mouseup="onMouseUp"
    @mousemove="onMouseMove" @mouseout="onMouseOut"/>
  </div>
</template>

<script>
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
      radiusRatio:0.75,
      joystickElement:null,  
      isMouseDown: false,
      canvasRefreshRate: 1.0, //Hz
      operatorCommandInterval: 1000, //ms
    }
  },
  methods: {
    init() {
      this.loopIntervalId = setInterval(function() {
        this.setCanvasSize();
        this.findCenterCanvas();
        this.drawCanvas();
      }.bind(this), 1000 / this.canvasRefreshRate);
      
      this.positionChangeIntervalId = setInterval(function() {
        this.emitJoystickPosition();
      }.bind(this), this.operatorCommandInterval);
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
      var rect = this.canvas.getBoundingClientRect();      
      this.x = event.clientX - rect.left;
      this.y = event.clientY - rect.top;

      let centerX = this.getCenterX();
      let centerY = this.getCenterY();  
      let deltaX = this.x - centerX;
      let deltaY = this.y - centerY;
      let radius = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
      let maxRadius = this.getCanvasRadius() - this.getJoystickRadius();

      if (radius > maxRadius)
      {
        let ratio = maxRadius / radius;
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
      }
      else {
        this.context.fillStyle = '#000000';
      }      

      this.context.beginPath();
      this.context.arc(this.x, this.y, this.getJoystickRadius(), 0, 2 * Math.PI);
      this.context.fill();
    },
    drawBackground() {
      let centerX = this.getCenterX();
      let centerY = this.getCenterY();

      let radius = this.getCanvasRadius();

      //Draw the background circle
      this.context.fillStyle = '#87CEEB';
      this.context.beginPath();
      this.context.arc(centerX, centerY, radius, 0, 2 * Math.PI);
      this.context.fill();

      
      let pointOffset = radius / 8;
      let halfPointOffset = pointOffset / 2;

      //draw center cross
      this.context.lineWidth = 2;
      this.context.strokeStyle = '#4682B4';
      this.context.beginPath();
      this.context.moveTo(centerX, centerY - pointOffset);
      this.context.lineTo(centerX, centerY + pointOffset);
      this.context.stroke();

      this.context.beginPath();
      this.context.moveTo(centerX - pointOffset, centerY);
      this.context.lineTo(centerX+ pointOffset, centerY);
      this.context.stroke();


      //draw the up triangle
      let upTriangleStartY = centerY - 3 * radius / 4;

      this.context.fillStyle = '#4682B4';
      this.context.beginPath();
      this.context.moveTo(centerX, upTriangleStartY);
      this.context.lineTo(centerX - halfPointOffset, upTriangleStartY + pointOffset);
      this.context.lineTo(centerX + halfPointOffset, upTriangleStartY + pointOffset);
      this.context.fill();

      //draw the down triangle
      let downTriangleStartY = centerY + 3 * radius / 4;

      this.context.beginPath();
      this.context.moveTo(centerX, downTriangleStartY);
      this.context.lineTo(centerX - halfPointOffset, downTriangleStartY - pointOffset);
      this.context.lineTo(centerX + halfPointOffset, downTriangleStartY - pointOffset);
      this.context.fill();

      //draw the left triangle
      let leftTriangleStartX = centerX - 3 * radius / 4;

      this.context.beginPath();
      this.context.moveTo(leftTriangleStartX, centerY);
      this.context.lineTo(leftTriangleStartX + pointOffset, centerY - halfPointOffset);
      this.context.lineTo(leftTriangleStartX + pointOffset, centerY + halfPointOffset);
      this.context.fill();

      //draw the right triangle
      let rightTriangleStartX = centerX + 3 * radius / 4;

      this.context.beginPath();
      this.context.moveTo(rightTriangleStartX, centerY);
      this.context.lineTo(rightTriangleStartX - pointOffset, centerY - halfPointOffset);
      this.context.lineTo(rightTriangleStartX - pointOffset, centerY + halfPointOffset);
      this.context.fill();
    },
    setCanvasSize(){
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
      return this.radiusRatio*Math.min(this.canvas.width, this.canvas.height) / 2;
    },
    getJoystickRadius() {
      return this.getCanvasRadius() / 4;
    },
    emitJoystickPosition() {
      let event = {
        x: (this.x - this.getCenterX()) * this.absoluteMaxX / (this.getCanvasRadius() - this.getJoystickRadius()),
        y: (this.y - this.getCenterY()) * this.absoluteMaxY / (this.getCanvasRadius() - this.getJoystickRadius()),
      };
      this.bus.$emit('joystick-position-change', event)
    },
  },
  mounted() {
    this.joystickElement = document.getElementById('joystick');
    this.canvas = this.$refs.canvas;
    this.context= this.canvas.getContext('2d');
    this.init();    
  },
  destroyed() {
    clearInterval(this.loopIntervalId);
    clearInterval(this.positionChangeIntervalId);
  }
}
</script>

<style>
.inner-joystick-container{
  padding:10px;
  height: 100%;
  width: 100%;
}
</style>
