<template>
  <div class="h-100 w-100">
    <navbar
      :my-id="myId"
      :robot-list="robotList"
    />
    <div
      style="height:480px;width:720px;"
    >
      <video-box
        :video-id="cameraId"
        :show="true"
      />
    </div>
    <div
      style="height:480px;width:720px;"
    >
      <video-box
        :video-id="mapId"
        :show="true"
      />
    </div>
  </div>
</template>

<script>
import { mapState } from 'vuex';
import Navbar from './NavbarNext';
import VideoBox from './VideoBox';

export default {
  name: 'impl',
  components: {
    Navbar,
    VideoBox,
  },
  computed: mapState({
    myId: state => state.myId,
    robotId: state => state.robotId,
    robotList: state => state.robotList,
    cameraId: state => state.htmlElement.cameraId,
    mapId: state => state.htmlElement.mapId,
  }),
  mounted() {
    this.$store.commit('setCameraHTMLElement', document.getElementById(this.cameraId));
    this.$store.commit('setMapHTMLElement', document.getElementById(this.mapId));
    this.$store.dispatch('connectToServer');
  },
};
</script>

<style scoped>

</style>
