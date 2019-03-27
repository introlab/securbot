<template>
  <!-- Video stream box -->
  <div
    id="VideoStream"
    class="w-100 h-100 m-auto position-relative bg-black-sb shadow-sb">
    <div class="w-100 h-100 position-relative d-inline-block">
      <video
        v-show="show"
        :id="VideoId"
        class="mw-100 h-100"/>
    </div>
  </div>
</template>

<script>
/*
* Author(s): Edouard Legare <edouard.legare@usherbrooke.ca>
* File :  VideoBox.vue
* Desc :  Vue SFC that makes a box containing a video element that will fit
*         the size given by the element in which it called (in the caller
*         component). It keeps the aspect ratio of the video and adapt the
*         the rest of the box (black bars) to fit. It takes 2 props, an id
*         for the video element so a feed can be set to it and a boolean show
*         to show or hid the video. It is used as a widget.
*
* Dependencies :
*       -Bootstrap-Vue
*
*/

// Export
export default {
  name: 'video-box',
  props: ['VideoId', 'show'],
};
</script>

<style>
</style>
