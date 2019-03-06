<template>
  <div class='waypoint-size'>
    <h1>Waypoints</h1>
    <div class='inner'> 
        <waypoint MapId='map' ShowMap='true' class='waypoint-container'/>
    </div>
  </div>
</template>

<script>
import Waypoint from './Waypoint.vue'

export default {
  name: 'demo-waypoint',
  components: {
      Waypoint
  },
  data(){
      return{
          localStream: null,
          selfStreamElement: null,
      }
  },
  methods:{
    connect() {
      easyrtc.enableDebug(false);
      console.log("Initializing.");
      easyrtc.enableVideo(true);
      easyrtc.enableAudio(false);
      easyrtc.enableVideoReceive(true);
      easyrtc.enableAudioReceive(false);
    
      easyrtc.initMediaSource(
        function(){
            this.localStream = easyrtc.getLocalStream();
            console.log(this.localStream);
            easyrtc.setVideoObjectSrc(this.selfStreamElement, this.localStream);
            console.log("Stream set, you are connected...");
        }.bind(this), this.loginFailure);
    },
    loginFailure(errorCode, message) {
      easyrtc.showError(errorCode, message);
    },
  },
    mounted() {
    this.selfStreamElement = document.getElementById("map");
    this.connect();
  },
  destroyed() {
    if (this.operatorEasyrtcId !== null) {
      easyrtc.hangupAll();
      easyrtc.disconnect();
    }
  }
};
</script>

<style>
.waypoint-size{
  width: 100%;
  height: 800px;
  position: fixed;
}
.inner{
    width: 100%;
    height: 100%;
    margin: auto;
    position: relative;
}
.waypoint-container{
    width: 100%;
    height: 100%;
    margin: auto;
}
</style>
