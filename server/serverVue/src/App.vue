<template>
  <div id="app">
    <h1>APP Vue.js</h1>
    <Joystick/>
    <Waypoint/>
    <Connection/>
    <VideoStream/>
  </div>
</template>

<script>
import Joystick from './components/Joystick.vue'
import Waypoint from './components/Waypoint.vue'
import Connection from './components/Connection.vue'
import VideoStream from './components/VideoStream.vue'

export default {
  name: 'app',
  components: {
    Joystick,
    Waypoint,
    Connection,
    VideoStream
  }
}
</script>

<style>
#app {
  font-family: 'Avenir', Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
  margin-top: 60px;
}
</style>
