'use strict';


//IDs
const webClientId = "client";
const securBotId = "securbot";
const room = "teleoperation";

function easyRTCConfiguration() {
//ASSURING NO AUDIO/VIDEO OF THE WEBCLIENT SHARED
easyRTC.enableAudio(false);
easyRTC.enableVideo(false);

//DATACHANNELS 
//Enabling data channels for teleoperation commands (must be done by both peers and before calling)
easyRTC.enableDataChannels(true);

//Listening events concerning availability of usage of particular peer's datachannel
easyRTC.setDataChannelOpenListener(function (securBotId) {
	 console.log("SecurBot datachannel is open");
}); 

//Listening also if datachannel of particular peer is closed
easyRTC.setDataChannelCloseListener(
	function(securBotId) {
		 console.log("SecurBot datachannel is/has been closed");
});

/*
//Listening for a response from robot peer on datachannel
easyRTC.setPeerListener(function(securBotId, null, msgData, null) {
	if( msgType === "commands" ) { 
		console.log( sendersEasyrtcid + msgData.command);
	}
});
*/

//Listening for errors
easyRTC.setOnError( function(erEvent) { 
	console.log(errEvent.errorText);
});
}


//
//INIT : Callback will be called whenever somebody else connects to or disconnects from "teleoperation"
function initTeleop() {
	easyRTC.setRoomOccupantListener(loggedInListener);
	easyRTC.easyApp(room, null ,["caller"],
		function(webClientId){
			console.log("Your Id is " + webClientId);
		}
	);
}

//LIST LOGGED IN PEERS IN ROOM
//TODO : Change the labels so it's not the easyRTCId but rather more permanent identifiers (names, titles, etc.)
function loggedInListener(room, otherPeers){
	var otherClientDiv = document.getElementById("otherClients");
	while (otherClientDiv.hasChildNodes()){//otherClientDiv = document.getElementById("otherClients")
		otherClientDiv.removeChild(otherClientDiv.lastChild);
	}
	for(var i in otherPeers) {
		var button = document.createElement("button");
		button.onclick = function(easyRTCId){
			return function() {
				performCall(easyRTCId);
			}	
		}(i);

		label = document.createTextNode(i);
		button.appendChild(label);
		otherClientDiv.appendChild(button);
	}
}

//CALLING
function performCall(easyRTCId){
	easyRTC.call(
		securBotId,
		function(securBotId) {  console.log("Completed call to " + securBotId);},
		function(errorMessage) { console.log("err:" + errorMessage);},
		function(accepted, bywho){
			console.log((accepted? "accepted":"rejected")+ "by " + bywho);
		}
	);
}

//SENDING MESSAGE P2P
function sendData(commandStr){
easyRTC.sendDataP2P(securBotId,"commandString", {
	command: commandStr
});
}

//VERIFYING CONNECTION
function checkNbPeersConnected(){
	console.log("You are connected to " + easyRTC.getConnectionCount() + " peer(s).");
}

//HANGING UP
function hangUp(){
	easyRTC.hangup(securBotId);
}

//DISCONNECTION
function disconnect(){
	easyRTC.disconnect();
}
