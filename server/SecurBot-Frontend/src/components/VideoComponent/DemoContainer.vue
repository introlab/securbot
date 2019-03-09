<template>
  <!-- Video stream container demo, shows how to call the video-box component and set a video
  stream in it, also allow to quick test of the video-box component -->
    <div class="center-element full-width">
        <h1>Video Component</h1>
        <div class="outer-video-box custom-width">
            <video-box VideoId="self-video-stream" :show="showSelf"/>
        </div>
        <div class="outer-video-box custom-width">
            <video-box VideoId="robot-video-stream" :show="showRobot"/>
        </div>  
    </div>
</template>

<script>
// Import component(s)
import VideoBox from './VideoBox.vue'

// Export
export default {
    name: 'demo-container',
    components: {
        VideoBox
    },
    data(){
    return {
        showSelf:true,
        showRobot:true,
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
        function(){
            this.localStream = easyrtc.getLocalStream();
            easyrtc.setVideoObjectSrc(this.selfStreamElement, this.localStream);
            easyrtc.connect("easyrtc.securbot", this.loginSuccess, this.loginFailure);
            console.log("Stream set, you are connected...");
        }.bind(this), this.loginFailure);
    },
    handleRoomOccupantChange(roomName, occupants, isPrimary) {
      this.otherId = null;
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
/* Some styles, will probably be changed */
.center-element{
    text-align: center
}
.full-width{
    width: 100%;
    height: 1000px;
}
.custom-width{
  margin: auto;
  width: 50%;
  height: 40%;
}
.outer-video-box{
    margin-top: 20px
}
</style>