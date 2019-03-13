<template>
  <div id="operator-layout">
    <div id="nav-bar">
  <!-- As a link -->
      <b-navbar toggleable="sm" variant="dark" type="primary">
        <b-navbar-brand href="#">SecureBot</b-navbar-brand>

        <b-navbar-toggle target="nav_collapse" />

        <b-collapse is-nav id="nav_collapse">
          <b-navbar-nav>
            <b-nav-item href="#">Teleoperation</b-nav-item>
            <b-nav-item href="#">Patrol Planner</b-nav-item>
            <b-nav-item href="#">Logs</b-nav-item>
          </b-navbar-nav>
        </b-collapse>
      </b-navbar>
    </div>

    <b-jumbotron head="SecureBot Vue.js Application" id="layout" :fluid="fluidState" :container-fluid="fluidState" bg-variant="gray-dark" border-variant="gray">
      <b-row :v-show="showTeleop" class="teleop-layout">
            <b-col sm="9">
              <div class="self-video-container">
                <video-box VideoId="self-video-stream" :show="showSelf"/>
              </div>
            </b-col>
            <b-col sm="3">
              <b-row class="joystick-element">
                <div class="outer-joystick-container">
                  <div class="inner-joystick-container">
                  </div>
                </div>
                <!--<joystick width="200px" height="200px" :absolute-max-x="1" :absolute-max-y="1" :bus="teleopBus"/>-->
              </b-row>
              <b-row class="connection-element">
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

import Vue from 'vue'

export default {

  name: 'layout',
  data(){
    return{
      fluidState:true,
      showTeleop:true,
      showWaypoint:false,
      showSelf:true,
      peerId:null,
      selfEasyrtcid:null,
      selfStreamElement:null,
      peerStreamElement:null,
      localStream:null,
      teleopBus: new Vue(),
    }
  },
  components: {
    VideoBox,
    Joystick,
    Connection,
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
      //to implement
      console.log("Joystick position sent");
    },
  },
  mounted() {
    this.selfStreamElement = document.getElementById("self-video-stream");
    //this.robotStreamElement = document.getElementById("robot-video-stream");
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
.self-video-container{
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
.outer-joystick-container{
  width: 100%;
  padding-top: 100%;
}
.inner-joystick-container{
  position: absolute;
  top: 0;
  left: 0;
  bottom: 0;
  right: 0;
  background-color: red;
}
.connection-element{
  height: 70%;
}
</style>
