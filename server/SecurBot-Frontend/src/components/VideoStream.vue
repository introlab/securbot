<template>
  <div id="VideoStream">
    <h1>VideoStream</h1>
    <div class="stream-container">
      <div class="self">
        <video :id="SelfVideoId" class="video-stream-box"/>
      </div>
      <div class="robot">
        <video :id="RobotVideoId" class="video-stream-box"/>
      </div>
    </div>
    <img src="/static/logo.png" alt="LOGO" v-show="showLogo"/>
  </div>
</template>

<script>
export default {
  name: 'video-stream',
  props: ['SelfVideoId','RobotVideoId','showLogo']
};
</script>

<style>

</style>