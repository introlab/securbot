<template>
  <div>
    <!--
    <div style="height:540px;width:960px;background-color:black;">
      <video-box
        :show="true"
        video-id="local-stream"/>
    </div>
    -->
    <div style="height:20px;width:960px;background-color:white;" />
    <div style="height:540px;width:960px;background-color:black;">
      <video-box
        :show="true"
        video-id="remote-stream" />
    </div>
    <div>
      <connection
        :self-id="selfEasyrtcid"
        :peers-table="testPeerTable"
        :bus="busBus" />
    </div>
  </div>
</template>

<script>
/*
  Page to test API by simulating an operator without all the other control.
  This page does not get video or audio, it just sets a data channel.
  When the simulator are done, this page needs to get remove (from index too)
*/
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
    // this.localElement = document.getElementById('local-stream');
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
      easyrtc.enableDebug(false);
      console.log('Initializing...');
      easyrtc.enableVideo(false);
      easyrtc.enableAudio(false);
      easyrtc.enableVideoReceive(true);
      easyrtc.enableAudioReceive(true);
      easyrtc.enableDataChannels(true);

      easyrtc.setDataChannelOpenListener(this.dataOpenListenerCB);
      easyrtc.setDataChannelCloseListener(this.dataCloseListenerCB);
      easyrtc.setPeerListener(this.handleData);

      easyrtc.setRoomOccupantListener(this.handleRoomOccupantChange);
      easyrtc.setStreamAcceptor(this.acceptPeerVideo);
      easyrtc.setOnStreamClosed(this.closePeerVideo);
      easyrtc.setAcceptChecker(this.acceptCall);

      easyrtc.setRoomApiField('default', 'type', 'robot_testing');

      easyrtc.connect('easyrtc.securbot', this.loginSuccess, this.loginFailure);
      console.log('Connected...');
    },
    handleRoomOccupantChange(roomName, occupants, isPrimary) {
      this.testPeerTable = [];
      if (occupants !== null) {
        // eslint-disable-next-line guard-for-in
        for (const occupant in occupants) {
          const peer = { peerName: occupant, peerId: occupant };
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
