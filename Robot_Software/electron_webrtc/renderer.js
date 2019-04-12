let operatorID = null
let virtualDevicesName = ['virtual_map', 'virtual_camera'];
let streamNames = [];

ipc.on('rosdata', (emitter, data) => {
    console.log(data)
    if(operatorID != null)
        easyrtc.sendDataP2P(operatorID, 'rosdata', data);
})

easyrtc.setStreamAcceptor( function(callerEasyrtcid, stream) {
    operatorID = callerEasyrtcid
    var video = document.getElementById('caller');
    easyrtc.setVideoObjectSrc(video, stream);
});

 easyrtc.setOnStreamClosed( function (callerEasyrtcid) {
     operatorID = null
    easyrtc.setVideoObjectSrc(document.getElementById('caller'), "");
});

function get_video_id(label) {
    return new Promise((resolve, reject) => {
        easyrtc.getVideoSourceList(device => {

            console.log(`Looking for ${device}...`);

            var videoSource = device.find(source => {
                return source.label.toString().trim() === label.trim()
            })

            if (videoSource == undefined) {
                console.log(`[${label}] video not found`)
                reject("Desired video stream not found")
                return
            }

            console.log(`Found [${label}] stream`)
            resolve(videoSource.id)
        })
    })
}

function dataCallback(easyrtcid, msgType, msgData) {
    console.log(msgData)
    ipc.send('msg', msgData)
}

function goalReceivedCallback(sourceId, msgType, goalJsonString) {
    console.log("Received new nav goal: " + goalJsonString)
    ipc.send('goal', goalJsonString)
}

function teleopCallback(easyrtcid, msgType, msgData) {
    console.log(msgData);
    ipc.send('msg', msgData)
}

function streamRequestCallback(easyrtcid, msgType, msgData) {
    console.log(`Received request of type ${msgType} for ${msgData}`)
    if(msgData === "map" || msgData === "camera") {
        easyrtc.addStreamToCall(easyrtcid, msgData);
    }
}

function fetchParameters() {

    return new Promise((resolve, reject) => {
        ipc.once('parameters_response', (event, params) => {
            resolve(params)
        })

        ipc.send('parameters_request')
    })
}

async function my_init() {

    var parameters = await fetchParameters()

    easyrtc.setRoomApiField('default', 'type', 'robot');
    easyrtc.setSocketUrl(parameters.webRtcServerUrl);
    easyrtc.setRoomOccupantListener(loggedInListener);

    easyrtc.enableVideo(true)
    easyrtc.enableAudio(false)

    easyrtc.enableVideoReceive(false);
    easyrtc.enableAudioReceive(false);
    easyrtc.enableDataChannels(true);

    easyrtc.setPeerListener(dataCallback, 'msg')
    easyrtc.setPeerListener(goalReceivedCallback, 'nav-goal')
    easyrtc.setPeerListener(teleopCallback, 'joystick-position')
    easyrtc.setPeerListener(streamRequestCallback, 'request-feed')

    easyrtc.setAcceptChecker(acceptCall)

    var connectSuccess = function(myId) {
        console.log("My easyrtcid is " + myId);
    }
    var connectFailure = function(errorCode, errText) {
        console.log(errText);
    }

    let isConnected = false;
    for (deviceName of virtualDevicesName) {
        get_video_id(deviceName).then(videoId => {
            easyrtc.setVideoSource(videoId)
    
            let streamName = deviceName.split('_')[1];
    
            streamNames.push(streamName);
    
            easyrtc.initMediaSource(
                  function(){        // success callback
                      // var selfVideo = document.getElementById("self");
                      // easyrtc.setVideoObjectSrc(selfVideo, easyrtc.getLocalStream());
                      if(!isConnected){
                        easyrtc.connect("easyrtc.securbot", connectSuccess, connectFailure);
                      }
                  },
                  connectFailure,
                  streamName
            );
        })
    }
 }


function loggedInListener(roomName, otherPeers) {
    var otherClientDiv = document.getElementById('otherClients');
    while (otherClientDiv.hasChildNodes()) {
        otherClientDiv.removeChild(otherClientDiv.lastChild);
    }
    for(var i in otherPeers) {
        var button = document.createElement('button');
        button.onclick = function(easyrtcid) {
            return function() {
                performCall(easyrtcid);
            }
        }(i);

        label = document.createTextNode(i);
        button.appendChild(label);
        otherClientDiv.appendChild(button);
    }
}

// This should one day accept every operator connection, but for now it only accept the first one to call.
function acceptCall(easyrtcid, acceptor) {
    if(operatorID === null) {
        operatorID = easyrtcid;
        console.log(`Accepting call from ${easyrtcid}`);
        acceptor(true, streamNames);
    }
}


function performCall(easyrtcid) {
    easyrtc.call(
       easyrtcid,
       function(easyrtcid) { console.log("completed call to " + easyrtcid);},
       function(errorCode, errorText) { console.log("err:" + errorText);},
       function(accepted, bywho) {
          console.log((accepted?"accepted":"rejected")+ " by " + bywho);
       }
   );
}


window.onload = () => { my_init() }
