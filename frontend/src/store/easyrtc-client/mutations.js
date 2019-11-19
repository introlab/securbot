export default {
  setServerConnState(state, serverState) {
    state.connectionState.server = serverState;
  },
  connected(state) {
    state.isConnected = true;
  },
  connectingToRobot(state) {
    state.connectionState.robot = 'connecting';
  },
  connectedToRobot(state) {
    state.connectionState.robot = 'connected';
  },
  lostRobot(state) {
    state.connectionState.robot = 'lost';
  },
  failedToConnectToRobot(state) {
    state.connectionState.robot = 'failed';
  },
  disconnectedFromRobot(state) {
    state.connectionState.robot = 'disconnected';
  },
  setMyId(state, id) {
    state.myId = id;
  },
  resetMyId(state) {
    state.myId = '';
  },
  setRobotId(state, id) {
    if (id) {
      state.robotId = id;
    }
  },
  resetRobotId(state) {
    if (state.robotId) {
      state.robotId = '';
    }
  },
  setCameraStream(state, stream) {
    if (stream) {
      state.cameraStream = stream;
    }
  },
  clearCameraStream(state) {
    if (state.cameraStream) {
      state.cameraStream = undefined;
    }
  },
  setMapStream(state, stream) {
    if (stream) {
      state.mapStream = stream;
    }
  },
  clearMapStream(state) {
    if (state.mapStream) {
      state.mapStream = undefined;
    }
  },
  addRobotToList(state, robot) {
    state.robotList.push(robot);
  },
  clearRobotList(state) {
    state.robotList = [];
  },
  enableDataChannel(state) {
    state.isDataChannelAvailable = true;
  },
  disableDataChannel(state) {
    state.isDataChannelAvailable = false;
  },
  setRobotStatus(state, status) {
    state.robotStatus = status;
  },
  setPatrolStatus(state, status) {
    state.patrolStatus = status;
  },
};
