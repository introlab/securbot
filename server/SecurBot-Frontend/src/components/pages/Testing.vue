<template>
  <div style="height:250px;width:250px;background-color:red;"/>
</template>

<script>
/* global easyrtc */

export default {
  name: 'testing',
  data() {
    return {
      // Rename variables
      peerId: null,
      isConnected: null,
      isInit: false,
      isDataChannelAvailable: false,
      selfEasyrtcid: null,
      localStreams: ['camera', 'map'],
      cameraStreamElement: null,
      mapStreamElement: null,
      patrolMapStreamElement: null,
      cameraStream: null,
      mapStream: null,
    };
  },
  /*
    mounted() : On component mounted, use to get and initialise
  */
  mounted() {
    easyrtc.initMediaSource(this.connect, this.connect);
  },
  // On component destroy, hangup and disconnect
  destroyed() {
    if (this.operatorEasyrtcId !== null) {
      this.isInit = false;
      easyrtc.hangupAll();
      easyrtc.disconnect();
    }
  },
  methods: {
    connect() {
      easyrtc.enableDebug(false);
      console.log('Initializing...');
      easyrtc.enableVideo(true);
      easyrtc.enableAudio(false);
      easyrtc.enableVideoReceive(false);
      easyrtc.enableAudioReceive(false);
      easyrtc.enableDataChannels(true);

      easyrtc.setDataChannelOpenListener(this.dataOpenListenerCB);
      easyrtc.setDataChannelCloseListener(this.dataCloseListenerCB);
      easyrtc.setPeerListener(this.handleData);
      easyrtc.setAcceptChecker(this.acceptCall);

      easyrtc.setRoomApiField('default', 'type', 'robot_testing');

      easyrtc.getVideoSourceList((videoSources) => {
        console.log('Getting video feed...');
        // eslint-disable-next-line no-plusplus
        for (let i = 0; i < videoSources.length; i++) {
          const videoSource = videoSources[i];
          const streamName = videoSource.label.toLowerCase();

          easyrtc.setVideoSource(videoSource.id);
          // eslint-disable-next-line no-loop-func
          easyrtc.initMediaSource(() => {
            this.handleMediaSourceInit(streamName);
          }, this.loginFailure, streamName);
        }
        console.log(this.localStreams);
        easyrtc.connect('easyrtc.securbot', this.loginSuccess, this.loginFailure);
      });
    },
    handleMediaSourceInit(streamName) {
      const stream = easyrtc.getLocalStream(streamName);
      this.localStreams[streamName] = stream;
    },
    loginSuccess(easyrtcid) {
      console.warn(`I am ${easyrtc.idToName(easyrtcid)}`);
      this.selfEasyrtcid = easyrtcid;
    },
    loginFailure(errorCode, message) {
      easyrtc.showError(errorCode, message);
    },
    acceptCall(easyrtcid, acceptor) {
      acceptor(true, this.localStreams);
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

