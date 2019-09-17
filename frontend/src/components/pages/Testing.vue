<template>
  <div
    class="h-100 w-100"
  >
    <h2>Component Testing</h2>
    <div
      style="height:480px; width:704px;"
    >
      <waypoint-overlay
        component-name="test-component"
      />
    </div>
  </div>
</template>

<script>
import { mapState } from 'vuex';
import WaypointOverlay from '../widget/Videobox&WaypointOverlay';

/* global easyrtc */

export default {
  name: 'testing',
  components: {
    WaypointOverlay,
  },
  data() {
    return {
      vid: null,
    };
  },
  computed: mapState({
    mapStream: state => state.mapStream,
  }),
  mounted() {
    this.vid = document.getElementById('test-component-videobox');
    this.$nextTick(() => {
      easyrtc.setVideoObjectSrc(this.vid, this.mapStream);
    });
  },
};
</script>

<style scoped>

</style>
