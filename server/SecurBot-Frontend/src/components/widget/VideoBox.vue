<template>
  <!-- Video stream box -->
  <div id="VideoStream" class="w-100 h-100 m-auto position-relative bg-black-sb shadow-sb">
    <div class="w-100 h-100 position-relative d-inline-block">
      <video :id="VideoId" class="mw-100 h-100" v-show="show"/>
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
</style>