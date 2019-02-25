easyrtc.setStreamAcceptor( function(callerEasyrtcid, stream) {
    var video = document.getElementById('caller');
    easyrtc.setVideoObjectSrc(video, stream);
});

 easyrtc.setOnStreamClosed( function (callerEasyrtcid) {
    easyrtc.setVideoObjectSrc(document.getElementById('caller'), "");
});


function my_init() {
    easyrtc.setSocketUrl('http://localhost:8080');
    easyrtc.setRoomOccupantListener( loggedInListener);
    var connectSuccess = function(myId) {
        console.log("My easyrtcid is " + myId);
    }
    var connectFailure = function(errorCode, errText) {
        console.log(errText);
    }
    easyrtc.initMediaSource(
          function(){        // success callback
              var selfVideo = document.getElementById("self");
              easyrtc.setVideoObjectSrc(selfVideo, easyrtc.getLocalStream());
              easyrtc.connect("Company_Chat_Line", connectSuccess, connectFailure);
          },
          connectFailure
    );
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