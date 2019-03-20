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
/*
* Author(s):  Edouard Legare <edouard.legare@usherbrooke.ca>
* File :  Layout.vue
* Desc :  Vue SFC that is the main routing point. All the routing is done by
*         this component (layout = parent, other pages = children). It has a 
*         navigation bar for the routing that also contains the connection 
*         component in a drop-down menu (simplify the connection process for
*         the user) and set the page height (currently the viewport height
*         minus the height the navbar). This component is the one managing
*         all the easyRTC necessary for the application. It communicates with
*         the children component with a bus.
*
* Dependencies : 
*       -Connection.vue
*       -Vue (node-module)
*       -Bootstrap-Vue
*
*/

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
    Connection,
  },
  methods:{
    /*
      General: most of the function here are for the easyRTC client
      To Do :
        -(SEC-346) Add the on router change function (event)
        -(SEC-365) Add function to set the remote feeds to video element on router change for the CURRENT page. 
                   Aka, only set the feed for the html element in the current viewed page, ignore the other.
        -(SEC-365) Add function to clear the remote feeds from video element on router change
        -(SEC-344) Clean code and make it fits standards
    */

    /*
      connect(): function managing init and connection to the easyrtc server
      To Do:
        -(SEC-364) Add data channel and it callback
        -(SEC-304) Remove local stream get/set
    */
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
            easyrtc.setVideoObjectSrc(this.selfStreamElement, this.localStream);
            //easyrtc.setVideoObjectSrc(this.self2StreamElement, this.localStream);
            easyrtc.connect("easyrtc.securbot", this.loginSuccess, this.loginFailure);
            console.log("Stream set, you are connected...");
        }.bind(this), this.loginFailure);
    },
    /*
      handleRoomOccupantChange(roomName, occupants, isPrimary) : Callback for the setRoomOccupantListener easyRTC function
        Desc : Trigger on occupants change in the room, give easyRTC id of all occupant
        To Do:
          -Clean function
          -(SEC-365) Get occupants name and id for peer list
    */
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
    /* 
      performCall(occupantId): function used to call someone in the room with its id
        To Do:
          -Clean function
          -(SEC-365) Create callback functions use by call outside of this function.
          -(SEC-304) Manage with event the different outcome
    */
    performCall(occupantId){
      console.log("Calling the chosen occupant : " + occupantId);
      easyrtc.hangupAll();

      //This should be defined outside
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

      //This should be defined outside
      var successCB = function() {
        console.warn("Call accepted...");
        this.teleopBus.$emit('connection-changed',"connected");
        this.peerId = occupantId;
      }.bind(this);

      //This should be defined outside
      var failureCB = function(errCode, errMessage) {
        console.warn("Call failed : " + errCode + " | " + errMessage);
        this.teleopBus.$emit('connection-changed',"failed");
        this.peerId = null;
      }.bind(this);

      easyrtc.call(occupantId, successCB, failureCB, acceptedCB);
    },
    /*
      loginSuccess(easyrtcid): Callback for connect success to the room
        Desc: When connection to room is successful, save self id
    */
    loginSuccess(easyrtcid) {
      console.warn("I am " + easyrtc.idToName(easyrtcid));
      this.selfEasyrtcid = easyrtcid;
    },
    /*
      loginFailure(errorCode, message): Callback for connect failure to the room
        Desc: When connection to room is successful, save self id 
    */
    loginFailure(errorCode, message) {
      easyrtc.showError(errorCode, message);
    },
    /*
      acceptPeerVideo(easyrtcid, stream): Callback for setStreamAcceptor() function
        Desc: This function called after a successful call
        To Do:
          -(SEC-365) Get the remote feeds from occupants
          -(SEC-365) Set feed(s) for the current page (Call the set feeds function)
    */
    acceptPeerVideo(easyrtcid, stream){
      easyrtc.setVideoObjectSrc(this.robotStreamElement, stream);
    },
    /*
      acceptPeerVideo(easyrtcid, stream): Callback for setOnStreamClosed() function
        Desc: Use to clear the last frame of a video feed that closed for reason out of our control (other client responsible)
    */
    closePeerVideo(easyrtcid){
      this.peerId = null;
      easyrtc.setVideoObjectSrc(this.robotStreamElement, this.peerId);
    },
    /*
      acceptCall(easyrtcid, acceptor): callback for setAcceptChecker() function
        Desc: When called, refuse the call, an operator cannot be called
        To Do:
          -(SEC-304) Always refused, cannot be called as an operator
    */
    acceptCall(easyrtcid, acceptor){
      console.log("You've been called...");
      if(easyrtc.getConnectionCount() > 0 ) 
      {
          easyrtc.hangupAll();
      }
      acceptor(true);
    },
    /*
      onJoystickPositionChange(): on event "joystick-position-change", this function is called
        Desc: Send the JSON string received through the data channel.
        To Do:
          -(SEC-364) Call the data sending function and give it the string
    */
    onJoystickPositionChange(){
      //console.log("Joystick position sent");
    },
    /*
      log(event) : on event "peer-connection", this function is called
      Desc: Receive an id and perform the call on this id if not already connected. 
            If connected and the id is the one of the connected peer, disconnect for it.
      To Do:
        -Rename the function and the parameter
    */
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
  /*
    mounted() : On component mounted, use to get and initialise
      To Do:
        -(SEC-365) - Make a function that gets the html elements and call it here.
  */
  mounted() {
    this.selfStreamElement = document.getElementById("self-video-stream");
    this.self2StreamElement = document.getElementById("map");
    this.robotStreamElement = document.getElementById("robot-video-stream");

    this.teleopBus.$on('peer-connection', this.log);
    this.teleopBus.$on('joystick-position-change', this.onJoystickPositionChange);


    this.connect();
  },
  //On component destroy, hangup and disconnect
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
