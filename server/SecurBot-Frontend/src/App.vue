<template>
  <div id="app">
    <h1>APP Vue.js</h1>
    <Joystick/>
    <Waypoint/>
    <Connection/>
    <VideoStream SelfVideoId="self-video-stream" RobotVideoId="robot-video-stream" :showLogo="showSelfVideo"/>
  </div>
</template>

<script>
import Joystick from './components/Joystick.vue'
import Waypoint from './components/Waypoint.vue'
import Connection from './components/Connection.vue'
import VideoStream from './components/VideoStream.vue'

export default {
  name: 'app',
  components: {
    Joystick,
    Waypoint,
    Connection,
    VideoStream
  },
  data(){
    return {
      showSelfVideo:true,
      otherId:null,
      selfEasyrtcid:null,
      selfStreamElement:null,
      robotStreamElement:null,
      localStream:null
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

      easyrtc.setRoomOccupantListener(this.handleRoomOccupantChange);
      easyrtc.setStreamAcceptor(this.acceptOtherVideo);
      easyrtc.setOnStreamClosed(this.closeOtherVideo);
      easyrtc.setAcceptChecker(this.acceptCall);
      easyrtc.setSocketUrl(":8080");
      easyrtc.initMediaSource(
        function(){        // success callback
            this.localStream = easyrtc.getLocalStream();
            easyrtc.setVideoObjectSrc(this.selfStreamElement, this.localStream);
            easyrtc.connect("easyrtc.securbot", this.loginSuccess, this.loginFailure);
            console.log("Stream set, you are connected...");
        }.bind(this), this.loginFailure);
    },
    handleRoomOccupantChange(roomName, occupants, isPrimary) {
      for(var easyrtcId in occupants)
      {
        if(this.otherId === null)
        {
          this.otherId = easyrtcId;
          this.performCall(this.otherId);
        }
        else
        {
          console.warn("Only one occupant is accepted for the moment...");
        }
      }
      
    },
    performCall(occupantId){
      easyrtc.hangupAll();
      var acceptedCB = function(accepted, easyrtcid) {
        if( !accepted ){
          console.warn("Call refused...");
        }
      };
      var successCB = function() {
          console.warn("Call accepted...");
      };
      var failureCB = function() {
          console.warn("Call failed...");
      };
      easyrtc.call(occupantId, successCB, failureCB, acceptedCB);
    },
    loginSuccess(easyrtcid) {
      this.selfEasyrtcid = easyrtcid;
      console.warn("I am " + easyrtc.idToName(easyrtcid));
    },
    loginFailure(errorCode, message) {
      easyrtc.showError(errorCode, message);
    },
    acceptOtherVideo(easyrtcid, stream){
      easyrtc.setVideoObjectSrc(this.robotStreamElement, stream);
    },
    closeOtherVideo(easyrtcid){
      this.otherId = null;
      easyrtc.setVideoObjectSrc(this.robotStreamElement, this.otherId);
    },
    acceptCall(easyrtcid, acceptor){
      if(easyrtc.getConnectionCount() > 0 ) 
      {
          easyrtc.hangupAll();
      }
      acceptor=true;
    }
  },
  mounted() {
    this.selfStreamElement = document.getElementById("self-video-stream");
    this.robotStreamElement = document.getElementById("robot-video-stream");
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
#app {
  font-family: 'Avenir', Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
  margin-top: 60px;
}
#self-video-stream, #robot-video-strea{
  position:relative;
  width: 480px;
  height: 360px;
}

</style>
