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
ipc.on('rosdata', (emitter, data) => {
  console.log(data);
  if (operatorID != null) { easyrtc.sendDataP2P(operatorID, 'rosdata', data); }
});

/**
 * Callback for the patrol-plan data channel from easyrtc.
 * @callback patrolReceivedCallback
 * @param {number} easyrtcId - Id of the peer sending data.
 * @param {String} msgType - Data channel the data are comming from.
 * @param {String} patrolJsonString - JSON string of the patrol data.
 */
function patrolReceivedCallback(easyrtcId, msgType, patrolJsonString) {
  console.log(`Received new patrol plan: ${patrolJsonString}`);
  ipc.send('patrol-plan', patrolJsonString);
}

/**
 * Callback for the joystick-position data channel from easyrtc.
 * @callback teleopCallback
 * @param {number} easyrtc - Id of the peer sending data.
 * @param {String} msgType - Data channel the data are comming from.
 * @param {String} msgData - JSON string of the teleop datas.
 */
function teleopCallback(easyrtcid, msgType, msgData) {
  console.log(msgData);
  ipc.send('msg', msgData);
}

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
  easyrtc.setRoomApiField('default', 'type', `robot-${robotName}`);
  easyrtc.setSocketUrl(webrtcServerURL);

  easyrtc.setAutoInitUserMedia(false);

  easyrtc.enableVideo(true);
  easyrtc.enableAudio(false);

  easyrtc.enableVideoReceive(false);
  easyrtc.enableAudioReceive(false);
  easyrtc.enableDataChannels(true);

  easyrtc.setPeerListener((id, type, data) => {
    console.log(`Receive ${data} from ${id} of type ${type}`);
  });
  easyrtc.setPeerListener(patrolReceivedCallback, 'patrol-plan');
  easyrtc.setPeerListener(teleopCallback, 'joystick-position');

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
