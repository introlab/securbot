<template>
  <!-- Video stream box -->
  <div id="VideoStream" class="video-container">
    <div class="inner-video-container">
      <video :id="VideoId" class="video-size" v-show="show"/>
    </div>
  </div>
</template>

<script>
// Description of the component :
// Needs an id for the html video element and a bool for the visibility of the element
// To set the video stream you need the html element (document.getElementById("VideoId"))
// Use easyrtc.setVideoObjectSrc(videoElement, stream) to set the stream

// Export
export default {
  name: 'video-box',
  props: ['VideoId','show']
};
</script>

<style>
/* Some css element, subject to change */
.video-container{
  width: 100%;
  height: 100%;
  margin: auto;
  position: relative;
  background-color: black;
  box-shadow: 0 4px 8px 0 rgba(0, 0, 0, 0.5), 0 6px 20px 0 rgba(0, 54, 5, 0.19);
}
.inner-video-container{
  width: 100%;
  height: 100%;
  display: inline-block;
  position: relative;
}
.video-size{
    max-width: 100%;
    height: 100% !important;
}
</style>