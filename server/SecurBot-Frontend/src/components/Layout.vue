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
            <b-nav-item to="teleop">
              Teleoperation
            </b-nav-item>
            <b-nav-item to="patrol">
              Patrol Planner
            </b-nav-item>
            <b-nav-item to="logs">
              Logs
            </b-nav-item>
          </b-navbar-nav>
          <b-navbar-nav class="ml-auto">
            <b-nav-item-dropdown
              text="Connect to Robot"
              right>
              <div class="px-2 py-1">
                <connection
                  :self-id="selfEasyrtcid"
                  :peers-table="peerTable"
                  :bus="teleopBus" />
              </div>
            </b-nav-item-dropdown>
          </b-navbar-nav>
        </b-collapse>
      </b-navbar>
    </div>
    <div style="height:calc(100% - 64px)">
      <router-view
        :bus="teleopBus"
        :router="routeBus" />
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
      fluidState: true,
      peerId: null,
      isConnected: null,
      isDataChannelAvailable: false,
      selfEasyrtcid: null,
      cameraStreamElement: null,
      mapStreamElement: null,
      patrolMapStreamElement: null,
      cameraStream: null,
      mapStream: null,
      localStream: null,
      teleopBus: new Vue(),
      routeBus: new Vue(),
      peerTable: [],
      joystickState: 'disable',
    };
  },
  // mounted() : On component mounted, use to get and initialise
  mounted() {
    this.teleopBus.$on('peer-connection', this.connectTo);
    this.teleopBus.$on('joystick-position-change', this.onJoystickPositionChange);
    this.teleopBus.$on('send-patrol', this.sendGoal);
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
    // connect(): function managing init and connection to the easyrtc server
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

      // Uncomment next line to use the dev server
      easyrtc.setSocketUrl('http://securbot.gel.usherbrooke.ca:8080');

      // Uncomment initialisation to use local stream for map (debugging only)
      // easyrtc.initMediaSource(() => {
      //   this.mapStream = easyrtc.getLocalStream();
      //   easyrtc.connect('easyrtc.securbot', this.loginSuccess, this.loginFailure);
      // }, this.loginFailure);

      // This is the production line, only comment if necessary for debugging
      easyrtc.connect('easyrtc.securbot', this.loginSuccess, this.loginFailure);

      console.log('You are connected...');
      this.setHTMLVideoStream();
    },
    /*
      handleRoomOccupantChange(roomName, occupants, isPrimary):
      Callback for the setRoomOccupantListener easyRTC function
        Desc : Trigger on occupants change in the room, give easyRTC id of all occupant
    */
    handleRoomOccupantChange(roomName, occupants, isPrimary) {
      this.peerTable = [];
      if (occupants !== null) {
        for (const occupant in occupants) {
          if (occupants[occupant].apiField.type.fieldValue.includes('robot')) {
            const peer = {
              peerName: occupants[occupant].apiField.type.fieldValue,
              peerId: occupant,
            };
            this.peerTable.push(peer);
          }
        }
      }
    },
    // performCall(occupantId): use to call someone in the room with its id
    performCall(occupantId) {
      easyrtc.hangupAll();
      console.log(`Calling the chosen occupant : ${occupantId}`);

      easyrtc.call(occupantId, this.callSuccessful, this.callFailure, this.callAccepted);
    },
    // callSuccessful(occupantId, mediaType): use to call someone in the room with its id
    callSuccessful(occupantId, mediaType) {
      console.warn(`Call to ${occupantId} was successful, here's the media: ${mediaType}`);
      if (mediaType === 'connection') {
        this.teleopBus.$emit('connection-changed', 'connected');
      }
    },
    // callFailure(errCode, errMessage): use to call someone in the room with its id
    callFailure(errCode, errMessage) {
      console.warn(`Call failed : ${errCode} | ${errMessage}`);
      this.teleopBus.$emit('connection-changed', 'failed');
      this.peerId = null;
    },
    // callAccepted(accepted, easyrtcid): use to call someone in the room with its id
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
      loginSuccess(easyrtcid): Callback for successfully connecting to the room
        Desc: When connection to room is successful, save self id
    */
    loginSuccess(easyrtcid) {
      console.warn(`I am ${easyrtc.idToName(easyrtcid)}`);
      this.selfEasyrtcid = easyrtcid;
    },
    /*
      loginFailure(errorCode, message): Callback for failure to connect to the room
        Desc: When connection to room is successful, save self id
    */
    loginFailure(errorCode, message) {
      easyrtc.showError(errorCode, message);
    },
    /*
      acceptPeerVideo(easyrtcid, stream): Callback for setStreamAcceptor() function
        Desc: This function called after a successful call
    */
    acceptPeerVideo(easyrtcid, stream, streamName) {
      console.log(`Stream received info, id : ${easyrtcid}, streamName : ${streamName}`);
      if (streamName === 'camera') {
        this.cameraStream = stream;
      } else if (streamName === 'map') {
        this.mapStream = stream;
      } else {
        console.warn('Unknown stream obtained...');
      }
      this.setHTMLVideoStream();
    },
    /*
      closePeerVideo(easyrtcid): Callback for setOnStreamClosed() function
        Desc: Use to clear the last frame of a video feed that closed for reason
              out of our control (other client responsible)
    */
    closePeerVideo(easyrtcid) {
      this.clearHTMLVideoStream();
    },
    /*
      acceptCall(easyrtcid, acceptor): callback for setAcceptChecker() function
        Desc: When called, refuse the call, an operator cannot be called
    */
    acceptCall(easyrtcid, acceptor) {
      console.log(`This id called : ${easyrtcid}, i'll only answer to ${this.peerId}`);
      if (easyrtcid === this.peerId) {
        acceptor(true);
      } else {
        acceptor(false);
      }
    },
    /*
      connectTo(event) : on event "peer-connection", this function is called
      Desc: Receive an id and perform the call on this id if not already connected.
            If connected and the id is the one of the connected peer, disconnect for it.
    */
    connectTo(easyrtcid) {
      console.log(`A connection change with ${easyrtcid} was asked...`);
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
    // dataOpenListenerCB(easyrtcid): Trigger on data channel opening with peer
    dataOpenListenerCB(easyrtcid) {
      console.warn(`Data channel open with ${easyrtcid}`);
      this.isDataChannelAvailable = true;
      this.joystickState = 'enable';
      this.teleopBus.$emit('on-joystick-state-changed', this.joystickState);

      // This request the stream from the robot so the operator doesn't have to have
      // a local stream to get the feed from the robot. It also allows to get both stream
      // from robot, which might have been a problem previously. They can be somewhere else.
      if (!this.mapStream) {
        console.log('Requesting the map stream from peer...');
        this.requestFeedFromPeer('map');
      }
      if (!this.cameraStream) {
        console.log('Requesting the camera stream from peer...');
        this.requestFeedFromPeer('camera');
      }
    },
    // dataCloseListenerCB(easyrtcid): Trigger on data channel closed with peer
    dataCloseListenerCB(easyrtcid) {
      this.isDataChannelAvailable = false;
      if (easyrtcid === this.peerId || !this.peerId) {
        this.peerId = null;
        this.teleopBus.$emit('connection-changed', 'disconnected');
        this.clearHTMLVideoStream();
      }
      this.joystickState = 'disable';
      this.teleopBus.$emit('on-joystick-state-changed', this.joystickState);
    },
    /*
      onJoystickPositionChange(): on event "joystick-position-change", this function is called
        Desc: Send the JSON string received through the data channel.
    */
    onJoystickPositionChange(data) {
      this.sendData(this.peerId, 'joystick-position', JSON.stringify(data));
    },
    /**
     * Sends a navigation waypoint to the robot in a JSON string
     */
    sendGoal(goalJsonString) {
      this.sendData(this.peerId, 'nav-goal', goalJsonString);
    },
    requestFeedFromPeer(feed) {
      this.sendData(this.peerId, 'request-feed', feed);
    },
    // sendData(peer, type, data): Send data through the data channel
    sendData(peer, type, data) {
      if (this.isDataChannelAvailable && peer) {
        easyrtc.sendDataP2P(peer, type, data);
      } else {
        console.warn('No data channel or peer available to send data...');
      }
    },
    // handleData(data): Handle datas coming through the data channel
    handleData(easyrtcid, type, data) {
      if (easyrtcid === this.peerId) {
        console.log(`Received ${data} of type ${type}...`);
      } else {
        console.log('Received data from someone else than the peer, ignoring it...');
      }
    },
    //
    verifyJoystickState() {
      if (this.isDataChannelAvailable) {
        this.joystickState = 'enable';
        this.teleopBus.$emit('on-joystick-state-changed', this.joystickState);
      }
    },
    // setHTMLVideoStream(): set the available video feed(s) to available html element(s)
    setHTMLVideoStream() {
      this.verifyJoystickState();

      this.getHTMLElements();

      if (this.cameraStreamElement && this.cameraStream) {
        console.log('Setting camera stream...');
        easyrtc.setVideoObjectSrc(this.cameraStreamElement, this.cameraStream);
      }
      if (this.mapStreamElement && this.mapStream) {
        console.log('Setting map stream...');
        easyrtc.setVideoObjectSrc(this.mapStreamElement, this.mapStream);
      }
      if (this.patrolMapStreamElement && this.mapStream) {
        console.log('Setting patrol stream...');
        easyrtc.setVideoObjectSrc(this.patrolMapStreamElement, this.mapStream);
      }
    },
    // clearHTMLVideoStream(): Clears all feeds in html and reset the feed variables
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
    // getHTMLElements() : Get HTML element of VideoBox component
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
