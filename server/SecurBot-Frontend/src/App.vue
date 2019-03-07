<template>
  <div id="app">
    <h1>SecurBot Vue.js APP</h1>
    <demo-container/>
  </div>
</template>

<script>
import DemoContainer from './components/VideoComponent/DemoContainer.vue'

export default {
  name: 'app',
  components: {
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
