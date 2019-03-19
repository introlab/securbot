<template>
  <div id="operator-layout" class='vh-100'>
    <div id="nav-bar" class="position-relative">
      <b-navbar toggleable="md" class="navbar-dark mb-0 bg-green-sb">
        <b-navbar-brand class="p-0">
          <div class="h-100" style="width:240px">
            <img src="../assets/SecurBotLogo.png" alt="SecurBot" class="logo mh-100 mw-100 align-middle"/>
          </div>
        </b-navbar-brand>
        <b-navbar-toggle target="nav_collapse" />
        <b-collapse is-nav id="nav_collapse">
          <b-navbar-nav>
            <b-nav-item to="teleop" active>Teleoperation</b-nav-item>
            <b-nav-item to="patrol">Patrol Planner</b-nav-item>
            <b-nav-item to="logs">Logs</b-nav-item>
          </b-navbar-nav>
          <b-navbar-nav class="ml-auto">
            <b-nav-item-dropdown text="Connect to Robot" right>
              <div class="px-2 py-1">
                <connection :selfId="selfEasyrtcid" :peersTable="testPeerTable" :bus="teleopBus"/>
              </div>
            </b-nav-item-dropdown>
          </b-navbar-nav>
        </b-collapse>
      </b-navbar>
    </div>
    <div style="height:calc(100% - 64px)">
      <router-view :bus="teleopBus"/>
    </div>
  </div>
</template>

<script>
import Connection from "./widget/Connection.vue";

import TeleopPage from "./pages/Teleop.vue";
import PatrolPage from "./pages/Patrol.vue";

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
    Connection,
    TeleopPage,
    PatrolPage,
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
      easyrtc.initMediaSource(
        function(){
            this.localStream = easyrtc.getLocalStream();
            //easyrtc.setVideoObjectSrc(this.selfStreamElement, this.localStream);
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
/* Changes to the bootstrap CSS */
.jumbotron{
  margin-bottom: 0;
}
.container-fluid{
  height: 100%
}
.navbar{
  min-height:64px;
}
/*Custom CSS element (SecurBot)*/
.shadow-sb{
  box-shadow: 0 4px 8px 0 rgba(0, 0, 0, 0.5), 0 6px 20px 0 rgba(0, 54, 5, 0.19);
}
.bg-black-sb{
  background-color: black;
}
.bg-green-sb{
  background-color:#00A759
}
.b-collapse-sb{
  border-collapse: collapse;
}
</style>
