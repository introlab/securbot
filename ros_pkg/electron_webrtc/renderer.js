/* global easyrtc, ipc, window */
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
 * Virtual devices names.
 * @type {Array}
 */
const virtualDevicesName = ['virtual_map', 'virtual_camera'];

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
  console.log("Received new patrol plan: " + patrolJsonString);
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
    easyrtc.addStreamToCall(easyrtcid, msgData);
  }
}

/**
 * Use to fetch the parameters from main
 * @function fetchParameters
 */
function fetchParameters() {
  return new Promise((resolve) => {
    ipc.once('parameters_response', (event, params) => {
      resolve(params);
    });

    ipc.send('parameters_request');
  });
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
    acceptor(true);
  } else {
    console.log(`Accepting call from ${easyrtcid}, this operator can only view me!`);
    acceptor(true);
  }
}

/**
 * Main function of the renderer; set all callback for easyrtc, connect the robot to the easyrtc
 * server and configure the 2 video stream.
 * @function myInit
 */
async function myInit() {
  const parameters = await fetchParameters();

  easyrtc.setRoomApiField('default', 'type', 'robot');
  easyrtc.setSocketUrl(parameters.webRtcServerUrl);

  easyrtc.enableVideo(true);
  easyrtc.enableAudio(false);

  easyrtc.enableVideoReceive(false);
  easyrtc.enableAudioReceive(false);
  easyrtc.enableDataChannels(true);

  easyrtc.setPeerListener(patrolReceivedCallback, 'patrol-plan');
  easyrtc.setPeerListener(teleopCallback, 'joystick-position');
  easyrtc.setPeerListener(streamRequestCallback, 'request-feed');

  easyrtc.setAcceptChecker(acceptCall);

  const connectSuccess = (myId) => {
    console.log(`My easyrtcid is ${myId}`);
  };
  const connectFailure = (errorCode, errText) => {
    console.log(errText);
  };

  let isConnected = false;

  easyrtc.getVideoSourceList((device) => {
    for (const deviceName of virtualDevicesName) {
      // eslint-disable-next-line max-len
      const videoSource = device.find(source => source.label.toString().trim() === deviceName.trim());

      if (videoSource) {
        console.log(`Found [${videoSource.label}] stream`);
        easyrtc.setVideoSource(videoSource.id);
        const streamName = videoSource.label.split('_')[1];

        streamNames.push(streamName);

        // eslint-disable-next-line no-loop-func
        easyrtc.initMediaSource(() => { // success callback
          console.log(`Initializing ${streamName}...`);
          if (!isConnected) {
            easyrtc.connect('easyrtc.securbot', connectSuccess, connectFailure);
            isConnected = true;
          }
        },
        connectFailure,
        streamName);
      }
    }
  });
}

/**
 * On load, start the renderer.
 */
window.onload = () => { myInit(); };
