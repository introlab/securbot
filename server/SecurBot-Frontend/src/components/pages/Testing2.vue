<template>
  <div>
    <div style="height:540px;width:960px;background-color:black;">
      <video-box
        :show="true"
        video-id="local-stream"/>
    </div>
    <div style="height:20px;width:960px;background-color:white;"/>
    <div style="height:540px;width:960px;background-color:black;">
      <video-box
        :show="true"
        video-id="remote-stream"/>
    </div>
    <div>
      <connection
        :self-id="selfEasyrtcid"
        :peers-table="testPeerTable"
        :bus="busBus"/>
    </div>
  </div>
</template>

<script>
/* global easyrtc */

import Vue from 'vue';

import VideoBox from '../widget/VideoBox';
import Connection from '../widget/Connection';

export default {
  name: 'testing',
  components: {
    VideoBox,
    Connection,
  },
  data() {
    return {
      // Rename variables
      peerId: null,
      isConnected: null,
      selfEasyrtcid: null,
      localStream: null,
      remoteStream: null,
      localStreams: ['camera', 'map'],
      localElement: null,
      remoteElement: null,
      testPeerTable: [],
      busBus: new Vue(),
    };
  },
  /*
    mounted() : On component mounted, use to get and initialise
  */
  mounted() {
    this.localElement = document.getElementById('local-stream');
    this.remoteElement = document.getElementById('remote-stream');
    this.busBus.$on('peer-connection', this.connectTo);

    this.connect();
  },
  // On component destroy, hangup and disconnect
  destroyed() {
    if (this.operatorEasyrtcId !== null) {
      easyrtc.hangupAll();
      easyrtc.disconnect();
    }
  },
  methods: {
    connect() {
      easyrtc.enableDebug(true);
      console.log('Initializing...');
      easyrtc.enableVideo(true);
      easyrtc.enableAudio(false);
      easyrtc.enableVideoReceive(false);
      easyrtc.enableAudioReceive(false);
      easyrtc.enableDataChannels(true);

      easyrtc.setDataChannelOpenListener(this.dataOpenListenerCB);
      easyrtc.setDataChannelCloseListener(this.dataCloseListenerCB);
      easyrtc.setPeerListener(this.handleData);

      easyrtc.setRoomOccupantListener(this.handleRoomOccupantChange);
      easyrtc.setStreamAcceptor(this.acceptPeerVideo);
      easyrtc.setOnStreamClosed(this.closePeerVideo);
      easyrtc.setAcceptChecker(this.acceptCall);

      easyrtc.setRoomApiField('default', 'type', 'robot_testing');

      // easyrtc.connect('easyrtc.securbot', this.loginSuccess, this.loginFailure);

      // eslint-disable-next-line no-loop-func
      easyrtc.initMediaSource(() => {
        this.localStream = easyrtc.getLocalStream();
        easyrtc.setVideoObjectSrc(this.localElement, this.localStream);
        easyrtc.connect('easyrtc.securbot', this.loginSuccess, this.loginFailure);
      }, this.loginFailure);
      console.log('Connected...');
    },
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
    connectTo(easyrtcid) {
      if (this.peerId === easyrtcid) {
        easyrtc.hangupAll();
        this.busBus.$emit('connection-changed', 'disconnected');
        this.peerId = null;
      } else if (this.peerId === null) {
        this.performCall(easyrtcid);
      } else {
        console.warn("The is an issue in the connection state handling... This shouldn't happen...");
      }
    },
    performCall(occupantId) {
      easyrtc.hangupAll();
      console.log(`Calling the chosen occupant : ${occupantId}`);

      easyrtc.call(occupantId, this.callSuccessful, this.callFailure, this.callAccepted);
    },
    callSuccessful(occupantId, mediaType) {
      console.warn(`Call to ${occupantId} was successful, here's the media: ${mediaType}`);
      if (mediaType === 'connection') {
        this.busBus.$emit('connection-changed', 'connected');
      }
    },
    callFailure(errCode, errMessage) {
      console.warn(`Call failed : ${errCode} | ${errMessage}`);
      this.busBus.$emit('connection-changed', 'failed');
      this.peerId = null;
    },
    callAccepted(accepted, easyrtcid) {
      console.warn(`Call was ${accepted} from ${easyrtcid}`);
      if (!accepted) {
        this.busBus.$emit('connection-changed', 'failed');
        this.peerId = null;
      } else {
        this.peerId = easyrtcid;
      }
    },
    loginSuccess(easyrtcid) {
      console.warn(`I am ${easyrtc.idToName(easyrtcid)}`);
      this.selfEasyrtcid = easyrtcid;
    },
    loginFailure(errorCode, message) {
      easyrtc.showError(errorCode, message);
    },
    acceptCall(easyrtcid, acceptor) {
      acceptor(true);
    },
    acceptPeerVideo(easyrtcid, stream) {
      this.remoteStream = stream;
      easyrtc.setVideoObjectSrc(this.remoteElement, this.remoteStream);
    },
    closePeerVideo(easyrtcid) {
      this.localStream = null;
      this.remoteStream = null;
      easyrtc.setVideoObjectSrc(this.localElement, '');
      easyrtc.setVideoObjectSrc(this.remoteElement, '');
    },
    dataOpenListenerCB(easyrtcid) {
      console.warn(`Data channel open with ${easyrtcid}`);
    },
    dataCloseListenerCB(easyrtcid) {
      console.warn(`Data channel close with ${easyrtcid}`);
    },
  },
};
</script>

<style>

</style>

