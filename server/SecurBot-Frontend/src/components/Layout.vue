<template>
  <div id="operator-layout">
    <div id="nav-bar">

      <b-navbar toggleable="sm" variant="success" type="dark" >
        <b-navbar-brand href="#">SecureBot</b-navbar-brand>

        <b-navbar-toggle target="nav_collapse" />

        <b-collapse is-nav id="nav_collapse">
          <b-navbar-nav tabs>
            <b-nav-item active>Teleoperation</b-nav-item>
            <b-nav-item>Patrol Planner</b-nav-item>
            <b-nav-item>Logs</b-nav-item>
          </b-navbar-nav>
          <b-navbar-nav class="ml-auto">
            <b-nav-item-dropdown text="Connect to Robot" right>
              <div class="connexion-container">
                <connection :selfId="selfEasyrtcid" :peersTable="testPeerTable" :bus="teleopBus"/>
              </div>
            </b-nav-item-dropdown>
          </b-navbar-nav>
        </b-collapse>

      </b-navbar>
    </div>

<!-- The jumbotron will be tranformed in a page (component with layout) in the future -->
    <b-jumbotron id="layout1" :fluid="fluidState" :container-fluid="fluidState" bg-variant="light">
      <b-row class="teleop-layout">
        <b-col lg="8" md="7" sm="6" style="max-height:100%">
          <div class="video-container">
            <video-box VideoId="self-video-stream" :show="showSelf"/>
          </div>
        </b-col>
        <b-col lg="4" md="5" sm="6" style="max-height:100%">
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

<!-- The jumbotron will be tranformed in a page (component with layout) in the future -->
<!-- To Note on this jumbotron: The way it is currently done is weird, the component that is call in it 
      have all of its layout already done inside of it. It should not be like that and need to changed. -->
    <b-jumbotron id="layout2" :fluid="fluidState" :container-fluid="fluidState" bg-variant="light">
        <waypoint mapId='map' showMap='true' :bus='teleopBus'/>
    </b-jumbotron>

  </div>
</template>

<script>
import VideoBox from "./operator/VideoBox.vue";
import Waypoint from "./operator/Waypoint.vue";
import Joystick from "./widget/Joystick.vue";
import Connection from "./widget/Connection.vue";

import Vue from 'vue'

export default {

  name: 'layout',
  data(){
    return{
      fluidState:true,
      showSelf:true,
      showRobot:true,
      peerId:null,
      isConnected:null,
      selfEasyrtcid:null,
      selfStreamElement:null,
      self2StreamElement:null,
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
    Waypoint,
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
            easyrtc.setVideoObjectSrc(this.self2StreamElement, this.localStream);
            easyrtc.connect("easyrtc.securbot", this.loginSuccess, this.loginFailure);
            console.log("Stream set, you are connected...");
        }.bind(this), this.loginFailure);
    },
    handleRoomOccupantChange(roomName, occupants, isPrimary) {
      //console.log(occupants);
      this.testPeerTable = [];
      if(occupants !== null){
        for(var occupant in occupants){
          var peer = {peerName:occupant,peerID:occupant};
          this.testPeerTable.push(peer);
        }
      }
      /*
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
      */
    },
    performCall(occupantId){
      console.log("Calling the chosen occupant : " + occupantId);
      easyrtc.hangupAll();

      var acceptedCB = function(accepted, easyrtcid) {
        console.log("Call was : " + accepted + " from " + easyrtcid)
        /*
        if( !accepted ){
          console.warn("Call refused...");
          this.teleopBus.$emit('connection-changed',"failed");
          this.peerId = null;
        }
        else{
          console.warn("Call accepted...");
        }
        */
      }.bind(this);

      var successCB = function() {
        console.warn("Call accepted...");
        this.teleopBus.$emit('connection-changed',"connected");
        this.peerId = occupantId;
      }.bind(this);

      var failureCB = function(errCode, errMessage) {
        console.warn("Call failed : " + errCode + " | " + errMessage);
        this.teleopBus.$emit('connection-changed',"failed");
        this.peerId = null;
      }.bind(this);

      easyrtc.call(occupantId, successCB, failureCB, acceptedCB);
    },
    loginSuccess(easyrtcid) {
      console.warn("I am " + easyrtc.idToName(easyrtcid));
      this.selfEasyrtcid = easyrtcid;
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
      console.log("You've been called...");
      if(easyrtc.getConnectionCount() > 0 ) 
      {
          easyrtc.hangupAll();
      }
      acceptor(true);
    },
    onJoystickPositionChange(){
      //console.log("Joystick position sent");
    },
    log(event){
      console.log("This is the connection event : " + event);
      if(this.peerId == event){
        easyrtc.hangupAll();
        console.log("Disconnection Accepted...");
        this.teleopBus.$emit('connection-changed',"disconnected");
        this.peerId = null;
      }
      else if(this.peerId === null){
        this.performCall(event);
      }
      else{
        console.warn("The is an issue in the connection state handling");
      }
    },
  },
  mounted() {
    this.selfStreamElement = document.getElementById("self-video-stream");
    this.self2StreamElement = document.getElementById("map");
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
.jumbotron{
  margin-bottom: 0;
}
.col{
  max-height: 100%;
}
.navbar{
  height:64px;
}
.b-navbar{
  margin-bottom: 0px;
}
.connexion-container{
  padding: 4px 8px;
}
.teleop-layout{
  height:calc(100vh - 192px);
}
.waypoint-layout{
  height: 700px;
  width: 100%;
  margin: auto;
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
  margin: auto;
  height: 50%;
  max-width: 300px;
}
.outer-joystick-container{
  position: relative;
  padding-top: 100%;
  margin: auto;
  width: 100%;
  height: 0;
}
.joystick-container{
  position: absolute;
  bottom: 0;
  left: 0;
  width: 100%;
  height : 100%;
  border: 2px solid dimgray;
  border-radius: 50%;
  box-shadow: 0 4px 8px 0 rgba(0, 0, 0, 0.5), 0 6px 20px 0 rgba(0, 54, 5, 0.19);
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
