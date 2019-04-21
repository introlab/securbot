<template>
  <!-- Video stream box -->
  <div
    id="VideoStream"
    class="w-100 h-100 m-auto position-relative bg-black-sb shadow-sb">
    <div class="w-100 h-100 position-relative">
      <video
        v-show="show"
        :id="videoId"
        class="w-100 h-100" />
    </div>
  </div>
</template>

<script>
/**
 * Vue SFC that makes a box containing a video element that will fit
 * the size given by the element in which it called (in the caller
 * component). It keeps the aspect ratio of the video and adapt the
 * the rest of the box (black bars) to fit. It takes 2 props, an id
 * for the video element so a feed can be set to it and a boolean show
 * to show or hid the video. It is used as a widget.
 * This component have the following dependency :
 * Bootstrap-Vue for styling.
 *
 *
 * @module widget/VideoBox
 * @vue-prop {String} videoId - Identifies source with exact name reference.
 * @vue-prop {boolean} show - Enable or disable the camera display.
 * /

/* Disabled comment documentation
 * Might use those eventually by forking jsdoc-vue-js so it can manage the author
 * and version tag correctly
 * @author Edouard Legare <edouard.legare@usherbrooke.ca>
 * @version 1.0.0
 */

export default {
  name: 'video-box',
  props: {
    videoId: {
      type: String,
      required: true,
    },
    show: {
      type: Boolean,
      required: true,
    },
  },
};
</script>

<style>
</style>
