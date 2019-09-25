export default {
  isConnected: false, // Connection:isConnected
  connectionState: {
    server: 'disconnected',
    robot: 'disconnected',
  },
  myId: '', // Layout:selfEasyrtcid
  robotId: '', // Layout:peerId
  cameraStream: undefined, // Layout:cameraStream
  mapStream: undefined, // Layout:mapStream | Teleop:showCamera|showMap
  robotList: [], // Layout:peerTable
  connecting: false, // Connection:waitingForConnectionState
  isDataChannelAvailable: false,
};
