<template>
  <div id="app">
    <h1>SecurBot Vue.js APP</h1>
    <joystick/>
    <waypoint/>
    <connection/>
    <demo-container/>
  </div>
</template>

<script>
import Joystick from './components/Joystick.vue'
import Waypoint from './components/Waypoint.vue'
import Connection from './components/Connection.vue'
import DemoContainer from './components/VideoComponent/DemoContainer.vue'

export default {
  name: 'app',
  components: {
    Joystick,
    Waypoint,
    Connection,
    DemoContainer
  },
};
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
