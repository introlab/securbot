/* global ipc, easyrtc, document, window */
let operatorID = null;

ipc.on('rosdata', (emitter, data) => {
  console.log(data);
  if (operatorID != null) { easyrtc.sendDataP2P(operatorID, 'rosdata', data); }
});

easyrtc.setStreamAcceptor((callerEasyrtcid, stream) => {
  operatorID = callerEasyrtcid;
  const video = document.getElementById('caller');
  easyrtc.setVideoObjectSrc(video, stream);
});

easyrtc.setOnStreamClosed((callerEasyrtcid) => {
  if (callerEasyrtcid === operatorID) {
    operatorID = null;
  }
});

function getVideoId(label) {
  return new Promise((resolve, reject) => {
    easyrtc.getVideoSourceList((list) => {
      const videoSource = list.find(source => source.label.toString().trim() === label.trim());

      if (videoSource === undefined) {
        console.log(`[${label}] video not found`);
        reject(new Error('Desired video stream not found'));
        return;
      }

      console.log('Found map stream');
      resolve(videoSource.id);
    });
  });
}

function dataCallback(easyrtcid, msgType, msgData) {
  console.log(msgData);
  ipc.send('msg', msgData);
}


function fetchParameters() {
  return new Promise((resolve, reject) => {
    ipc.once('parameters_response', (event, params) => {
      resolve(params);
    });

    ipc.send('parameters_request');
  });
}

async function mInit() {
  const parameters = await fetchParameters();

  console.log(`Attempting to connect to : ${parameters.webRtcServerUrl}`);
  easyrtc.setSocketUrl(parameters.webRtcServerUrl);
  const connectSuccess = (myId) => {
    console.log(`My easyrtcid is ${myId}`);
  };
  const connectFailure = (errorCode, errText) => {
    console.log(errText);
  };

  getVideoId(parameters.videoDeviceLabel).then((videoId) => {
    easyrtc.setVideoSource(videoId);
    easyrtc.enableDataChannels(true);
    easyrtc.enableAudio(false);
    easyrtc.setPeerListener(dataCallback, 'msg');

    easyrtc.initMediaSource(
      () => { // success callback
        const selfVideo = document.getElementById('self');
        easyrtc.setVideoObjectSrc(selfVideo, easyrtc.getLocalStream());
        easyrtc.connect('easyrtc.securbot', connectSuccess, connectFailure);
      },
      connectFailure,
    );
  });
}

window.onload = () => { mInit(); };
