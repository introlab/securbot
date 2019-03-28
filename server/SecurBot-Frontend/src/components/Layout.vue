<template>
  <div
    id="operator-layout"
    class="vh-100">
    <div
      id="nav-bar"
      class="position-relative">
      <b-navbar
        toggleable="md"
        class="navbar-dark mb-0 bg-green-sb">
        <b-navbar-brand class="p-0">
          <div
            class="h-100"
            style="width:240px">
            <img
              src="../assets/SecurBotLogo.png"
              alt="SecurBot"
              class="logo mh-100 mw-100 align-middle">
          </div>
        </b-navbar-brand>
        <b-navbar-toggle target="nav_collapse" />
        <b-collapse
          id="nav_collapse"
          is-nav>
          <b-navbar-nav>
            <b-nav-item
              to="teleop"
              active>Teleoperation</b-nav-item>
            <b-nav-item to="patrol">Patrol Planner</b-nav-item>
            <b-nav-item to="logs">Logs</b-nav-item>
          </b-navbar-nav>
          <b-navbar-nav class="ml-auto">
            <b-nav-item-dropdown
              text="Connect to Robot"
              right>
              <div class="px-2 py-1">
                <connection
                  :self-id="selfEasyrtcid"
                  :peers-table="testPeerTable"
                  :bus="teleopBus"/>
              </div>
            </b-nav-item-dropdown>
          </b-navbar-nav>
        </b-collapse>
      </b-navbar>
    </div>
    <div style="height:calc(100% - 64px)">
      <router-view
        :bus="teleopBus"
        :router="routeBus"/>
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
/* global easyrtc */

import Vue from 'vue';
import Connection from './widget/Connection';

export default {

  name: 'layout',
  components: {
    Connection,
  },
  data() {
    return {
      // Rename variables
      fluidState: true,
      peerId: null,
      isConnected: null,
      isInit: false,
      isDataChannelAvailable: false,
      selfEasyrtcid: null,
      cameraStreamElement: null,
      mapStreamElement: null,
      patrolMapStreamElement: null,
      localStream: null,
      cameraStream: null,
      mapStream: null,
      teleopBus: new Vue(),
      routeBus: new Vue(),
      testPeerTable: [{ peerName: 'Robot1', peerID: 'aogiyudlf' },
        { peerName: 'Robot2', peerID: 'fqw98rasd' }],
    };
  },
  /*
    mounted() : On component mounted, use to get and initialise
  */
  mounted() {
    this.teleopBus.$on('peer-connection', this.connectTo);
    this.teleopBus.$on('joystick-position-change', this.onJoystickPositionChange);
    this.routeBus.$on('mounted', this.setHTMLVideoStream);
    this.routeBus.$on('destroyed', this.clearHTMLVideoStream);

    this.connect();
  },
  // On component destroy, hangup and disconnect
  destroyed() {
    if (this.selfEasyrtcid !== null) {
      easyrtc.hangupAll();
      easyrtc.disconnect();
    }
  },
  methods: {
    /*
      connect(): function managing init and connection to the easyrtc server
    */
    connect() {
      easyrtc.enableDebug(false);
      console.log('Initializing...');
      easyrtc.enableVideo(false);
      easyrtc.enableAudio(false);
      easyrtc.enableVideoReceive(true);
      easyrtc.enableAudioReceive(false);
      easyrtc.enableDataChannels(true);

      easyrtc.setDataChannelOpenListener(this.dataOpenListenerCB);
      easyrtc.setDataChannelCloseListener(this.dataCloseListenerCB);
      easyrtc.setPeerListener(this.handleData);
      easyrtc.setRoomOccupantListener(this.handleRoomOccupantChange);
      easyrtc.setStreamAcceptor(this.acceptPeerVideo);
      easyrtc.setOnStreamClosed(this.closePeerVideo);
      easyrtc.setAcceptChecker(this.acceptCall);

      easyrtc.setRoomApiField('default', 'type', 'operator');

      easyrtc.connect('easyrtc.securbot', this.loginSuccess, this.loginFailure);

      console.log('You are connected...');

      this.setHTMLVideoStream();
    },
    /*
      handleRoomOccupantChange(roomName, occupants, isPrimary):
      Callback for the setRoomOccupantListener easyRTC function
        Desc : Trigger on occupants change in the room, give easyRTC id of all occupant
        To Do:
          -(SEC-365) Get occupants name and id for peer list
    */
    handleRoomOccupantChange(roomName, occupants, isPrimary) {
      this.testPeerTable = [];
      if (occupants !== null) {
        // eslint-disable-next-line guard-for-in
        for (const occupant in occupants) {
          const peer = { peerName: occupant, peerID: occupant };
          this.testPeerTable.push(peer);
        }
      }
    },
    /*
      performCall(occupantId): function used to call someone in the room with its id
    */
    performCall(occupantId) {
      easyrtc.hangupAll();
      console.log(`Calling the chosen occupant : ${occupantId}`);

      easyrtc.call(occupantId, this.callSuccessful, this.callFailure, this.callAccepted);
    },
    callSuccessful(occupantId, mediaType) {
      console.warn(`Call to ${occupantId} was successful, here's the media: ${mediaType}`);
      if (mediaType === 'connection') {
        this.teleopBus.$emit('connection-changed', 'connected');
      }
    },
    callFailure(errCode, errMessage) {
      console.warn(`Call failed : ${errCode} | ${errMessage}`);
      this.teleopBus.$emit('connection-changed', 'failed');
      this.peerId = null;
    },
    callAccepted(accepted, easyrtcid) {
      console.warn(`Call was ${accepted} from ${easyrtcid}`);
      if (!accepted) {
        this.teleopBus.$emit('connection-changed', 'failed');
        this.peerId = null;
      } else {
        this.peerId = easyrtcid;
      }
    },

    /*
      loginSuccess(easyrtcid): Callback for connect success to the room
        Desc: When connection to room is successful, save self id
    */
    loginSuccess(easyrtcid) {
      console.warn(`I am ${easyrtc.idToName(easyrtcid)}`);
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
    acceptPeerVideo(easyrtcid, stream, streamName) {
      console.log('Something Something');
      console.log(`Id: ${easyrtcid}, Stream Name:${streamName}`);
      this.setHTMLVideoStream();
    },
    /*
      closePeerVideo(easyrtcid): Callback for setOnStreamClosed() function
        Desc: Use to clear the last frame of a video feed that closed for reason
              out of our control (other client responsible)
    */
    closePeerVideo(easyrtcid) {
      this.peerId = null;
      this.clearHTMLVideoStream();
    },
    /*
      acceptCall(easyrtcid, acceptor): callback for setAcceptChecker() function
        Desc: When called, refuse the call, an operator cannot be called
    */
    acceptCall(easyrtcid, acceptor) {
      acceptor(false);
    },
    /*
      onJoystickPositionChange(): on event "joystick-position-change", this function is called
        Desc: Send the JSON string received through the data channel.
    */
    onJoystickPositionChange(data) {
      if (this.isDataChannelAvailable) {
        easyrtc.sendDataP2P(this.peerId, data);
      } else {
        console.warn('You should not be able to send data...');
      }
    },
    /*
      log(event) : on event "peer-connection", this function is called
      Desc: Receive an id and perform the call on this id if not already connected.
            If connected and the id is the one of the connected peer, disconnect for it.
    */
    connectTo(easyrtcid) {
      if (this.peerId === easyrtcid) {
        easyrtc.hangupAll();
        this.teleopBus.$emit('connection-changed', 'disconnected');
        this.peerId = null;
      } else if (this.peerId === null) {
        this.performCall(easyrtcid);
      } else {
        console.warn("The is an issue in the connection state handling... This shouldn't happen...");
      }
    },
    //
    dataOpenListenerCB(easyrtcid) {
      console.warn(`Data channel open with ${easyrtcid}`);
      this.isDataChannelAvailable = true;
      this.teleopBus.$emit('on-joystick-state-changed', 'enable');
    },
    //
    dataCloseListenerCB(easyrtcid) {
      console.warn(`Data channel close with ${easyrtcid}`);
      this.isDataChannelAvailable = false;
      this.teleopBus.$emit('on-joystick-state-changed', 'disable');
    },
    handleData(data) {
      console.log("Someday we'll do something with the data, but not today...");
    },
    // The next 2 functions need to change ?
    setHTMLVideoStream() {
      this.getHTMLElements();

      if (this.isInit) {
        if (this.cameraStreamElement && this.cameraStream) {
          easyrtc.setVideoObjectSrc(this.cameraStreamElement, this.cameraStream);
        }
        if (this.mapStreamElement && this.mapStream) {
          easyrtc.setVideoObjectSrc(this.mapStreamElement, this.mapStream);
        }
        if (this.patrolMapStreamElement && this.mapStream) {
          easyrtc.setVideoObjectSrc(this.patrolMapStreamElement, this.mapStream);
        }
      }
    },
    clearHTMLVideoStream() {
      if (this.cameraStreamElement) {
        easyrtc.setVideoObjectSrc(this.cameraStreamElement, '');
      }
      if (this.mapStreamElement) {
        easyrtc.setVideoObjectSrc(this.mapStreamElement, '');
      }
      if (this.patrolMapStreamElement) {
        easyrtc.setVideoObjectSrc(this.patrolMapStreamElement, '');
      }

      this.cameraStreamElement = null;
      this.mapStreamElement = null;
      this.patrolMapStreamElement = null;
    },
    // Get HTML element of VideoBox component
    getHTMLElements() {
      this.cameraStreamElement = document.getElementById('camera-stream');
      this.mapStreamElement = document.getElementById('map-stream');
      this.patrolMapStreamElement = document.getElementById('patrol-map-stream');
    },
  },
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
