<template>
  <div>
    <div style="height:540px;width:960px;background-color:black;">
      <video-box
        :show="true"
        video-id="local-stream"
      />
    </div>
    <div style="height:20px;width:960px;background-color:white;" />
    <div style="height:540px;width:960px;background-color:black;">
      <video-box
        :show="true"
        video-id="remote-stream"
      />
    </div>
    <div>
      <connection
        :self-id="selfEasyrtcid"
        :robot-list="testPeerTable"
        :bus="busBus"
      />
    </div>
  </div>
</template>

<script>
/*
  Page to test API by simulating a robot without all the other control.
  This page gets video feed from computer and sets a data channel.
  When the simulator are done, this page needs to get remove (from index too)
*/
/* global easyrtc */

import Vue from 'vue';

import VideoBox from '../widgets/VideoBox';
import Connection from '../widgets/Connection';

export default {
  name: 'testing2',
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
      remoteStream: null,
      localStreamNames: [],
      localStreams: {},
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
      easyrtc.enableDebug(false);
      console.log('Initializing...');
      easyrtc.setAutoInitUserMedia(false);
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

      const id = Math.round(Math.random() * 1000);
      easyrtc.setRoomApiField('default', 'type', `robot_testing_${id}`);
      easyrtc.setSocketUrl(process.env.VUE_APP_SERVER_URL);
      easyrtc.connect('easyrtc.securbot', this.loginSuccess, this.loginFailure);

      // eslint-disable-next-line no-loop-func
      easyrtc.getVideoSourceList((videoSources) => {
        for (let i = 0; i < videoSources.length; i++) {
          const videoSource = videoSources[i];

          const streamName = videoSource.label;
          this.localStreams[streamName] = videoSource.id;
          this.localStreamNames.push(streamName);

          easyrtc.setVideoSource(videoSource.id);
          // eslint-disable-next-line no-loop-func
          easyrtc.initMediaSource((stream) => {
            this.localStreams[streamName] = stream;

            if (streamName.includes('map')) {
              easyrtc.setVideoObjectSrc(this.remoteElement, stream);
            } else if (streamName.includes('camera')) {
              easyrtc.setVideoObjectSrc(this.localElement, stream);
            }
          }, this.loginFailure, streamName);
        }
        console.log('Connected...');
      });
    },
    handleRoomOccupantChange(roomName, occupants) {
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
      this.peerId = easyrtcid;
      acceptor(true, this.localStreamNames[0]);
    },
    acceptPeerVideo(easyrtcid, stream) {
      this.remoteStream = stream;
      easyrtc.setVideoObjectSrc(this.remoteElement, this.remoteStream);
    },
    closePeerVideo() {
      this.localStreams = [];
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
    handleData(easyrtcid, type, data) {
      if (easyrtcid === this.peerId && type === 'onDemandStream') {
        console.log(`${easyrtcid} requested the ${data} feed/stream...`);
        for (let i = 0; i < this.localStreamNames.length; i++) {
          if (this.localStreamNames[i].includes(data)) {
            easyrtc.addStreamToCall(easyrtcid, this.localStreamNames[i], (id, name) => {
              console.log(`${id} acknowledges receiving ${name}`);
            });
          }
        }
      } else if (easyrtcid === this.peerId) {
        console.log(`Received ${data} of type ${type}...`);
      } else {
        console.log('Received data from someone else than the peer, ignoring it...');
      }
    },
  },
};
</script>

<style>

</style>
