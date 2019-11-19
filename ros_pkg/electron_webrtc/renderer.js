/* global easyrtc, ipc, virtualDeviceNames, webrtcServerUrl, robotName */
/**
 * Renderer for this application.
 * @module Renderer
 *
 * @author Edouard Denomme <>
 * @author Cedric Godin <>
 * @author Edouard Legare <>
 */

/**
 * Keep track of the operator id.
 * @type {number}
 */
let operatorID = null;
let peerIDs = [];

/**
 * Array to keep track of the virtual devices corrected name.
 * @type {Array}
 */
const streamNames = [];

/**
 * Send data comming from ROS to the easyrtc server.
 * @method
 * @param {String} data - Data comming from ROS to send to server.
 * @listens rosdata
 */
ipc.on('robot-status', (_, data) => {
  let toRemove = [];
  peerIDs.forEach((peer) => {
    try {
      easyrtc.sendDataP2P(peer, 'robot-status', data);
    }
    catch(_) {
      toRemove.push(peer);
    }
  });
  peerIDs = peerIDs.filter(peer => toRemove.findIndex(item => item === peer));
});

/**
 * Store map size from robot
 */
let mapSize = {
  width: 0,
  height: 0
};
ipc.on('map-size', (_, data) => {
  mapSize = data;
})

/**
 * Callback for the request-feed data channel msg from easyrtc.
 * @callback streamRequestCallback
 * @param {number} easyrtc - Id of the peer sending data.
 * @param {String} msgType - Data channel the data are comming from.
 * @param {String} msgData - String of the requested string name.
 */
function streamRequestCallback(easyrtcid, msgType, msgData) {
  console.log(`Received request of type ${msgType} for ${msgData}`);
  if (msgData === 'map' || msgData === 'camera') {
    easyrtc.addStreamToCall(easyrtcid, msgData, (id, name) => {
      console.log(`Stream ${name} received by ${id}`);
    });
    console.log(`Stream ${msgData} added to call`);
  }
}

/**
 * Callback of the accept checker of easyrtc
 * @callback acceptCall
 * @param {number} easyrtcid - Id of the peer sending data.
 * @param {callback} acceptor - Need to be sets to access or refuse a call.
 */
function acceptCall(easyrtcid, acceptor) {
  peerIDs.push(easyrtcid);
  if (operatorID === null) {
    operatorID = easyrtcid;
    console.log(`Accepting call from ${easyrtcid}, this operator can control me!`);
    acceptor(true, streamNames[0]);
    setTimeout(() => streamRequestCallback(easyrtcid, 'fake-request', streamNames[1]), 1000);
  } else {
    console.log(`Accepting call from ${easyrtcid}, this operator can only view me!`);
    acceptor(true, streamNames[0]);
    setTimeout(() => streamRequestCallback(easyrtcid, 'fake-request', streamNames[1]), 1000);
  }
}

/**
 * Main function of the renderer; set all callback for easyrtc, connect the robot to the easyrtc
 * server and configure the 2 video stream.
 * @function myInit
 */
function myInit() {
  easyrtc.setRoomApiField('default', 'name', robotName);
  easyrtc.setRoomApiField('default', 'type', 'robot');
  easyrtc.setSocketUrl(webrtcServerURL);

  easyrtc.setAutoInitUserMedia(false);

  easyrtc.enableVideo(true);
  easyrtc.enableAudio(false);

  easyrtc.enableVideoReceive(false);
  easyrtc.enableAudioReceive(false);
  easyrtc.enableDataChannels(true);

  // Forward all received data to main process
  easyrtc.setPeerListener((_, type, data) => {
    ipc.send(type, data)
  });

  // Send map size when data channel opens
  easyrtc.setDataChannelOpenListener((easyrtcid) => {
    console.log('Data channel open');
    if (mapSize.height && mapSize.width) {
      easyrtc.sendDataP2P(easyrtcid, 'map-size', mapSize);
      console.log(`Sent map size ${mapSize.width}x${mapSize.height}`);
    }
  });

  easyrtc.setAcceptChecker(acceptCall);

  const connectSuccess = (myId) => {
    console.log(`My easyrtcid is ${myId}`);
  };
  const connectFailure = (errorCode, errText) => {
    console.log(errText);
  };

  let isConnected = false;

  console.log('Getting video sources');
  easyrtc.getVideoSourceList((device) => {
    console.log('Devices:');
    console.log(virtualDeviceNames);
    for (const deviceName of virtualDeviceNames) {
      console.log(`Finding ${deviceName} stream...`);
      // eslint-disable-next-line max-len
      const videoSource = device.find((source) => source.label.toString().trim() === deviceName.trim());

      if (videoSource) {
        console.log(`Found [${videoSource.label}] stream`);
        easyrtc.setVideoSource(videoSource.id);
        const streamName = videoSource.label.split('_')[1];

        streamNames.push(streamName);

        // eslint-disable-next-line no-loop-func
        console.log(`Initializing ${streamName}...`);
        // eslint-disable-next-line no-loop-func
        easyrtc.initMediaSource((stream) => { // success callback
          easyrtc.setVideoObjectSrc(document.getElementById(streamName), stream);
          console.log(`${streamName} initialized...`);
          if (!isConnected) {
            easyrtc.connect('easyrtc.securbot', connectSuccess, connectFailure);
            isConnected = true;
          }
        },
        connectFailure,
        streamName);
        console.log(`${streamName} added to easyrtc`);
      }
    }
  });
}

/**
 * On load, start the renderer.
 */
window.onload = () => { myInit(); };
