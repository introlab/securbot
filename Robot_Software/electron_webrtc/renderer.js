/* global easyrtc, ipc, window, document */
let operatorID = null;
const virtualDevicesName = ['virtual_map', 'virtual_camera'];
const streamNames = [];

ipc.on('rosdata', (emitter, data) => {
  console.log(data);
  if (operatorID != null) { easyrtc.sendDataP2P(operatorID, 'rosdata', data); }
});

easyrtc.setStreamAcceptor((callerEasyrtcid, stream) => {
  operatorID = callerEasyrtcid;
  const video = document.getElementById('caller');
  easyrtc.setVideoObjectSrc(video, stream);
});

function dataCallback(easyrtcid, msgType, msgData) {
  console.log(msgData);
  ipc.send('msg', msgData);
}

function goalReceivedCallback(sourceId, msgType, goalJsonString) {
  console.log(`Received new nav goal: ${goalJsonString}`);
  ipc.send('goal', goalJsonString);
}

function teleopCallback(easyrtcid, msgType, msgData) {
  console.log(msgData);
  ipc.send('msg', msgData);
}

function streamRequestCallback(easyrtcid, msgType, msgData) {
  console.log(`Received request of type ${msgType} for ${msgData}`);
  if (msgData === 'map' || msgData === 'camera') {
    easyrtc.addStreamToCall(easyrtcid, msgData);
  }
}

function fetchParameters() {
  return new Promise((resolve) => {
    ipc.once('parameters_response', (event, params) => {
      resolve(params);
    });

    ipc.send('parameters_request');
  });
}

// function loggedInListener(roomName, otherPeers) {
//   const otherClientDiv = document.getElementById('otherClients');
//   while (otherClientDiv.hasChildNodes()) {
//     otherClientDiv.removeChild(otherClientDiv.lastChild);
//   }
//   for (const i in otherPeers) {
//     const button = document.createElement('button');
//     button.onclick = (function (easyrtcid) {
//       return function () {
//         performCall(easyrtcid);
//       };
//     }(i));

//     label = document.createTextNode(i);
//     button.appendChild(label);
//     otherClientDiv.appendChild(button);
//   }
// }

function acceptCall(easyrtcid, acceptor) {
  if (operatorID === null) {
    operatorID = easyrtcid;
    console.log(`Accepting call from ${easyrtcid}`);
    acceptor(true);
  } else {
    acceptor(true);
  }
}

function getVideoSource(label) {
  return new Promise((resolve, reject) => {
    easyrtc.getVideoSourceList((device) => {
      console.log(`Looking for ${device}...`);

      const videoSource = device.find(source => source.label.toString().trim() === label.trim());

      if (videoSource === undefined) {
        console.log(`[${label}] video not found`);
        reject(new Error('The desired video stream cannot be found...'));
        return;
      }

      console.log(`Found [${label}] stream`);
      resolve(videoSource.id);
    });
  });
}

async function myInit() {
  const parameters = await fetchParameters();

  easyrtc.setRoomApiField('default', 'type', 'robot');
  easyrtc.setSocketUrl(parameters.webRtcServerUrl);

  easyrtc.enableVideo(true);
  easyrtc.enableAudio(false);

  easyrtc.enableVideoReceive(false);
  easyrtc.enableAudioReceive(false);
  easyrtc.enableDataChannels(true);

  easyrtc.setPeerListener(dataCallback, 'msg');
  easyrtc.setPeerListener(goalReceivedCallback, 'nav-goal');
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

window.onload = () => { myInit(); };
