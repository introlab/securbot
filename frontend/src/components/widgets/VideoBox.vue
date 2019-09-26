<template>
  <!-- Video Box widget -->
  <div
    id="VideoStream"
    class="w-100 h-100 m-auto position-relative bg-black-sb shadow-sb"
  >
    <!-- Video container -->
    <div class="w-100 h-100 position-relative">
      <!-- Video -->
      <video
        v-show="show"
        :id="videoId"
        class="w-100 h-100"
      />
    </div>
  </div>
</template>

<script>
/**
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
