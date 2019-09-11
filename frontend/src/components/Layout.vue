<template>
  <!-- Layout -->
  <div
    id="operator-layout"
    class="vh-100"
  >
    <!-- Navbar container-->
    <div
      id="sec-nav"
      class="position-relative"
    >
      <!-- Navbar -->
      <b-navbar
        toggleable="md"
        class="navbar-dark mb-0 bg-green-sb"
      >
        <!-- Brand -->
        <b-navbar-brand class="p-0">
          <div
            class="h-100"
            style="width:240px"
          >
            <img
              src="../assets/SecurBotLogo.png"
              alt="SecurBot"
              class="logo mh-100 mw-100 align-middle"
            >
          </div>
        </b-navbar-brand>
        <!-- Collapse Option -->
        <b-navbar-toggle target="nav_collapse" />
        <!-- Collapsable Navbar -->
        <b-collapse
          id="nav_collapse"
          is-nav
        >
          <!-- Navbar right side content -->
          <b-navbar-nav>
            <!-- Teleop -->
            <b-nav-item to="teleop">
              Teleoperation
            </b-nav-item>
            <!-- Patrol -->
            <b-nav-item to="patrol">
              Patrol Planner
            </b-nav-item>
            <!-- Event -->
            <b-nav-item to="logs">
              Logs
            </b-nav-item>
          </b-navbar-nav>
          <!-- Navbar left side content -->
          <b-navbar-nav class="ml-auto">
            <!-- Dropdown with connection widget -->
            <b-button
              variant="warning"
              @click="vuexTesting"
            >
              Test
            </b-button>
            <b-nav-item-dropdown
              text="Connect to Robot"
              right
            >
              <!-- Connection container -->
              <div class="px-2 py-1">
                <!-- Connection -->
                <connection
                  :self-id="myId"
                  :robot-list="robotList"
                  :bus="teleopBus"
                />
              </div>
            </b-nav-item-dropdown>
          </b-navbar-nav>
        </b-collapse>
      </b-navbar>
    </div>
    <!-- Main content (page) -->
    <div style="height:calc(100% - 64px)">
      <!-- Router to pages -->
      <router-view
        :bus="teleopBus"
        :router="routeBus"
      />
    </div>
  </div>
</template>

<script>
/**
 * Vue SFC that is the main routing point. All the routing is done by
 * this component (layout = parent, other pages = children). It has a
 * navigation bar for the routing that also contains the connection
 * component in a drop-down menu (simplify the connection process for
 * the user) and set the page height (currently the viewport height
 * minus the height the navbar). This component is the one managing
 * all the easyRTC necessary for the application. It communicates with
 * the children component with a bus. The layout has the following
 * dependencies : Connection component and Bootstrap-vue for styling
 * and HTML element.
 *
 * @module Layout
 * @vue-data {String} peerId - Id of the connected peer.
 * @vue-data {Boolean} isDataChannelAvailable - Keep track if the data channel is open.
 * @vue-data {HTMLVideoElement} cameraStreamElement - HTML video element to host camera.
 * @vue-data {HTMLVideoElement} mapStreamElement - HTML video element to host map.
 * @vue-data {HTMLVideoElement} patrolMapStreamElement - HTML video element to host map.
 * @vue-data {VideoTrack} cameraStream - VideoTrack given by easyrtc of the camera.
 * @vue-data {VideoTrack} mapStream - VideoTrack given by easyrtc of the camera.
 * @vue-data {Vue} teleopBus - General communication bus between component. Change name.
 * @vue-data {Vue} routeBus - Communication bus for the routing event.
 * @vue-data {String[]} peerTable - List of the robot in the room.
 * @vue-data {String} joystickState - Indicates if the joystick should be activable or not.
 * @vue-event {String} connection-changed - Use to communicate the state of connection
 * has changed to the other components.
 * @vue-event {String} on-joystick-state-changed - Use to communicate the joystick can
 * be used to the teleop page.
 */

/**
 * @author Edouard Legare <edouard.legare@usherbrooke.ca>
 */

/* global easyrtc */

import Vue from 'vue';
import { mapState } from 'vuex';
import Connection from './widget/Connection';

/**
 * Check to change the easyrtc module
 * Check what to do with the set/reset of the html element
 */

export default {
  name: 'layout',
  components: {
    Connection,
  },
  data() {
    return {
      isDataChannelAvailable: false,
      cameraStreamElement: null,
      mapStreamElement: null,
      patrolMapStreamElement: null,
      teleopBus: new Vue(),
      routeBus: new Vue(),
      joystickState: 'disable',
    };
  },
  computed: mapState({
    myId: state => state.myId,
    robotId: state => state.robotId,
    robotList: state => state.robotList,
    robotConnection: state => state.connectionState.robot,
    mapStream: state => state.mapStream,
    cameraStream: state => state.cameraStream,
  }),
  /**
   * On component mounted, use to get and initialise
   * @method
   */
  mounted() {
    this.teleopBus.$on('peer-connection', this.connectTo);
    this.teleopBus.$on('joystick-position-change', this.onJoystickPositionChange);
    this.teleopBus.$on('send-patrol', this.sendPatrol);
    this.routeBus.$on('mounted', this.setHTMLVideoStream);
    this.routeBus.$on('destroyed', this.clearHTMLVideoStream);

    this.connect();
  },
  /**
   * On component destroy, hangup and disconnect
   * @method
   */
  destroyed() {
    if (!this.myId) {
      easyrtc.hangupAll();
      easyrtc.disconnect();
    }
  },
  methods: {
    vuexTesting() {
      this.$store.dispatch('handleRobotsInRoom', this.$store.state.test);
    },
    /**
     * Function managing initialisation and connection to the easyrtc server
     * @method
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

      // Uncomment next line to use the dev server
      easyrtc.setSocketUrl(process.env.VUE_APP_SERVER_URL);

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
    /**
     * Callback for the handle of occupant in room.
     * @method
     * @param {String} roomName - Name of the room.
     * @param {Array} occupants - List of occupant in the room.
     * @param {Boolean} isPrimary
     */
    handleRoomOccupantChange(roomName, occupants) {
      console.log(occupants);
      if (Object.keys(occupants).length) {
        this.$store.dispatch('handleRobotsInRoom', occupants);
      }
    },
    /**
     * Use to call someone in the room with its id.
     * @method
     * @param {String} occupantId - Id to call.
     */
    performCall(occupantId) {
      easyrtc.hangupAll();
      console.log(`Calling the chosen occupant : ${occupantId}`);

      easyrtc.call(occupantId, this.callSuccessful, this.callFailure, this.callAccepted);
    },
    /**
     * Callback for a successful call.
     * @method
     * @param {String} occupantId - Id of the occupant that was called.
     * @param {String} mediaType - Type of media received (ex: AudioVideo)
     */
    callSuccessful(occupantId, mediaType) {
      console.warn(`Call to ${occupantId} was successful, here's the media: ${mediaType}`);
      if (mediaType === 'connection') {
        this.teleopBus.$emit('connection-changed', 'connected');
      }
    },
    /**
     * Callback for call failure.
     * @method
     * @param {Number} errCode - Error Code.
     * @param {String} errMessage - Error Message.
     */
    callFailure(errCode, errMessage) {
      console.warn(`Call failed : ${errCode} | ${errMessage}`);
      this.teleopBus.$emit('connection-changed', 'failed');
      this.$store.commit('resetRobotId');
    },
    /**
     * Callback for call accepted.
     * @method
     * @param {Boolean} accepted - If the call was accepted.
     * @param {String} easyrtcid - Id of the occupant that accepted the call.
     */
    callAccepted(accepted, easyrtcid) {
      console.warn(`Call was ${accepted} from ${easyrtcid}`);
      if (!accepted) {
        this.teleopBus.$emit('connection-changed', 'failed');
        this.$store.commit('resetRobotId');
      } else {
        this.$store.commit('setRobotId', easyrtcid);
      }
    },
    /**
     * Callback for successfully connecting to the room.
     * @method
     * @param {String} easyrtcid - Self id give by the server.
     */
    loginSuccess(easyrtcid) {
      console.warn(`I am ${easyrtc.idToName(easyrtcid)}`);
      this.$store.commit('setMyId', easyrtcid);
    },
    /**
     * Callback for failure to connect to the room.
     * @method
     * @param {Number} errorCode - Error code.
     * @param {String} message - Error message.
     */
    loginFailure(errorCode, message) {
      easyrtc.showError(errorCode, message);
    },
    /**
     * Callback for the setStreamAcceptor function.
     * @method
     * @param {String} easyrtcid - Id of the occupant giving the video stream.
     * @param {VideoTrack} stream - Track of the stream coming from the server.
     * @param {String} streamName - Name of the stream coming from the server.
     */
    acceptPeerVideo(easyrtcid, stream, streamName) {
      console.log(`Stream received info, id : ${easyrtcid}, streamName : ${streamName}`);
      if (streamName === 'camera') {
        this.$store.commit('setCameraStream', stream);
      } else if (streamName === 'map') {
        this.$store.commit('setMapStream', stream);
      } else {
        console.warn('Unknown stream passed...');
      }
      this.setHTMLVideoStream();
    },
    /**
     * Callback for the setOnStreamClosed function.
     * @method
     * @param {String} easyrtcid - Id of the occupant that disabled or lost its stream.
     */
    closePeerVideo() {
      this.clearHTMLVideoStream();
    },
    /**
     * Callback for the setAcceptChecker function.
     * @method
     * @param {String} easyrtcid - Id of the occupant calling.
     * @param {Callback} acceptor - Callback to accept the call.
     */
    acceptCall(easyrtcid, acceptor) {
      console.log(`This id called : ${easyrtcid}, i'll only answer to ${this.robotId}`);
      if (easyrtcid === this.robotId) {
        acceptor(true);
      } else {
        acceptor(false);
      }
    },
    /**
     * Callback of the "peer-connection"event.
     * @method
     * @param {String} easyrtcid - Id of the occupant to connect to.
     */
    connectTo(easyrtcid) {
      console.log(`A connection change with ${easyrtcid} was asked...`);
      if (this.robotConnection === 'connected') {
        easyrtc.hangupAll();
        this.teleopBus.$emit('connection-changed', 'disconnected');
        this.$store.commit('resetRobotId');
      } else if (this.robotId && this.robotConnection === 'connecting') {
        this.performCall(easyrtcid);
      } else {
        console.warn("The is an issue in the connection state handling... This shouldn't happen...");
      }
    },
    /**
     * Callback for the setDataChannelOpenListener function.
     * @method
     * @param {String} easyrtc - Id of the occupant that a channel was opened with.
     */
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
      setTimeout(() => {
        if (!this.cameraStream) {
          console.log('Requesting the camera stream from peer...');
          this.requestFeedFromPeer('camera');
        }
      }, 1000);
    },
    /**
     * Callback for the setDataChannelCloseListener function.
     * @method
     * @param {String} easyrtc - Id of the occupant that a channel was closed with.
     */
    dataCloseListenerCB(easyrtcid) {
      this.isDataChannelAvailable = false;
      if (easyrtcid === this.robotId || this.robotId) {
        this.$store.commit('resetRobotId');
        this.teleopBus.$emit('connection-changed', 'disconnected');
        this.clearHTMLVideoStream();
      }
      this.joystickState = 'disable';
      this.teleopBus.$emit('on-joystick-state-changed', this.joystickState);
    },
    /**
     * Callback of the "joystick-position-change" event.
     * @method
     * @param {Object} data - Teleop data.
     */
    onJoystickPositionChange(data) {
      this.sendData(this.robotId, 'joystick-position', JSON.stringify(data));
    },
    /**
     * Callback of the "send-patrol" event.
     * @method
     * @param {String} goalJsonString - Stringigify JSON of the patrol.
     */
    sendPatrol(goalJsonString) {
      this.sendData(this.robotId, 'patrol-plan', goalJsonString);
    },
    /**
     * Use to request a stream from the peer.
     * @method
     * @param {String} feed - Feed Name to request.
     */
    requestFeedFromPeer(feed) {
      this.sendData(this.robotId, 'request-feed', feed);
    },
    /**
     * Sends data to the robot using the data channel.
     * @method
     * @param {String} goalJsonString - Stringigify JSON of the patrol.
     * @param {String} type - Channel to send the data on.
     * @param {String} data - Stringify data to send.
     */
    sendData(peer, type, data) {
      if (this.isDataChannelAvailable && peer) {
        easyrtc.sendDataP2P(peer, type, data);
      } else {
        console.warn('No data channel or peer available to send data...');
      }
    },
    /**
     * Callback of the setPeerListener function. Not currently used.
     * @method
     * @param {String} easyrtcid - Peer id the datas are are coming from.
     * @param {String} type - Channel to data was received on.
     * @param {String} data - Data received.
     */
    handleData(easyrtcid, type, data) {
      if (easyrtcid === this.robotId) {
        console.log(`Received ${data} of type ${type}...`);
      } else {
        console.log('Received data from someone else than the peer, ignoring it...');
      }
    },
    /**
     * Verify if the joystick should be enable.
     * @method
     */
    verifyJoystickState() {
      if (this.isDataChannelAvailable) {
        this.joystickState = 'enable';
        this.teleopBus.$emit('on-joystick-state-changed', this.joystickState);
      }
    },
    /**
     * Sets the available video feed(s) to available html element(s).
     * @method
     */
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
    /**
     * Clears all feeds in html and reset the feed variables.
     * @method
     */
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
    /**
     * Get HTML element(s) of VideoBox component in the current page.
     */
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
/* #sec-nav a {
  font-weight: bold;
  color: white;
} */
#sec-nav a.router-link-exact-active {
  color: white;
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
