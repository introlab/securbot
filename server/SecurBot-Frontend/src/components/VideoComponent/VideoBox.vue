<template>
  <div id="VideoStream">
    <div class="video-container">
      <div class="inner-video-container video-size">
        <video class="video-size" :id="VideoId" v-show="show"/>
      </div>
    </div>
  </div>
</template>

<script>
export default {
  name: 'video-box',
  props: ['VideoId','show']
};
</script>

<style>
.video-container{
  width: 100%;
  margin: auto;
  position: relative;
}
.inner-video-container{
  margin: auto;
  position: relative;
  box-shadow: 0 4px 8px 0 rgba(0, 0, 0, 0.5), 0 6px 20px 0 rgba(0, 0, 0, 0.19);
}
.video-size{
    width: 480px;
    height: 360px;
}
</style>