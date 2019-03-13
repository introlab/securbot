<template>
  <div id="operator-layout">
    <div id="nav-bar">
  <!-- As a link -->
      <b-navbar toggleable="sm" variant="success" type="dark" >
        <b-navbar-brand href="#">SecureBot</b-navbar-brand>

        <b-navbar-toggle target="nav_collapse" />

        <b-collapse is-nav id="nav_collapse">
          <b-navbar-nav pills>
            <b-nav-item active>Teleoperation</b-nav-item>
            <b-nav-item>Patrol Planner</b-nav-item>
            <b-nav-item>Logs</b-nav-item>
          </b-navbar-nav>
        </b-collapse>
      </b-navbar>
    </div>

    <b-jumbotron id="layout" :fluid="fluidState" :container-fluid="fluidState" bg-variant="dark">
      <b-row :v-show="showTeleop" class="teleop-layout">
        <!--<demo-container/>-->
        <b-col sm="8">
          <div class="video-container">
            <video-box VideoId="self-video-stream" :show="showSelf"/>
          </div>
        </b-col>
        <b-col sm="4">
          <b-row class="robot-video-row">
            <div class="video-container">
              <video-box VideoId="robot-video-stream" :show="showRobot"/>
            </div>
          </b-row>
          <b-row class="joystick-row">
            <div class="outer-joystick-container">
              <div class="joystick-container">
                <joystick :absolute-max-x="1" :absolute-max-y="1" :bus="teleopBus"/>
              </div>
            </div>
          </b-row>
        </b-col>
      </b-row>
    </b-jumbotron>
  </div>
</template>

<script>
import VideoBox from "./operator/VideoBox.vue";
import Joystick from "./widget/Joystick.vue";
import Connection from "./widget/Connection.vue";

import DemoContainer from "./demo/DemoContainer.vue"

import Vue from 'vue'

export default {

  name: 'layout',
  data(){
    return{
      fluidState:true,
      showTeleop:true,
      showWaypoint:false,
      showSelf:true,
      showRobot:true,
      peerId:null,
      selfEasyrtcid:null,
      selfStreamElement:null,
      robotStreamElement:null,
      localStream:null,
      teleopBus: new Vue(),
      testPeerTable:[{peerName:"Robot1",peerID:"aogiyudlf"},
                {peerName:"Robot2",peerID:"fqw98rasd"}],
    }
  },
  components: {
    VideoBox,
    Joystick,
    Connection,
    DemoContainer,
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
      easyrtc.setStreamAcceptor(this.acceptPeerVideo);
      easyrtc.setOnStreamClosed(this.closePeerVideo);
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
      this.peerId = null;
      for(var easyrtcId in occupants)
      {
        if(this.peerId === null)
        {
          this.peerId = easyrtcId;
          this.performCall(this.peerId);
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
    acceptPeerVideo(easyrtcid, stream){
      easyrtc.setVideoObjectSrc(this.robotStreamElement, stream);
    },
    closePeerVideo(easyrtcid){
      this.peerId = null;
      easyrtc.setVideoObjectSrc(this.robotStreamElement, this.peerId);
    },
    acceptCall(easyrtcid, acceptor){
      if(easyrtc.getConnectionCount() > 0 ) 
      {
          easyrtc.hangupAll();
      }
      acceptor=true;
    },
    onJoystickPositionChange(){
      console.log("Joystick position sent");
    },
    log(event){
      console.log("Event!")
      console.log(event);
    },
  },
  mounted() {
    this.selfStreamElement = document.getElementById("self-video-stream");
    this.robotStreamElement = document.getElementById("robot-video-stream");

    this.teleopBus.$on('peer-connection', this.log);
    this.teleopBus.$on('joystick-position-change', this.onJoystickPositionChange);


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
#jumbo-content{
  height: 100%;
}
.b-navbar{
  margin-bottom: 0px;
}
.teleop-layout{
  height:700px;
}
.waypoint-layout{
  height: 700px;
}
.video-container{
  width: 100%;
  height: 100%;
  margin: auto;
  position: relative;
}
.full-row{
  height: 100%;
}
.half-row{
  height: 50%;
}
.robot-video-row{
  height: 50%;
  width: 100%;
  position: relative;
  margin: 0px;
  margin-bottom: 20px;
}
.joystick-row{
  position: relative;
  margin-top: auto;
  height: 30%;
  width: 100%;
  max-width: 300px;
  margin-left: auto;
  margin-right: auto;
}
.outer-joystick-container{
  position: relative;
  padding-top: 100%;
  width: 100%;
  height: 50%;
}
.joystick-container{
  position: absolute;
  bottom: 0;
  left: 0;
  width: 100%;
  height : 100%;
  border: 2px solid dimgray;
  border-radius: 50px;
}
.connection-row{
  position: relative;
  width: 100%;
  margin: 0px;
  margin-top: 20px;
}
.full{
  height: 100%;
  width: 100%;
}
.half{
  height: 50%;
  width: 50%;
}
</style>
