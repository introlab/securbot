/**
 * The EasyRTC-Client module mutations.
 *
 * @module EasyRTC-Client
 * @exports
 */
export default {
  /**
   * Sets the connection state with the server.
   * @param {Vuex} state - Vuex states
   * @param {String} serverState - The connection state with the server
   */
  setServerConnState(state, serverState) {
    state.connectionState.server = serverState;
  },
  /**
   * Returns true is the UI app is connected to a robot.
   * @param {Vuex} state - Vuex states
   */
  connected(state) {
    state.isConnected = true;
  },
  /**
   * Sets the robot connection state to connecting.
   * @param {Vuex} state - Vuex states
   */
  connectingToRobot(state) {
    state.connectionState.robot = 'connecting';
  },
  /**
   * Sets the robot connection state to connected.
   * @param {Vuex} state - Vuex states
   */
  connectedToRobot(state) {
    state.connectionState.robot = 'connected';
  },
  /**
   * Sets the robot connection state to lost.
   * @param {Vuex} state - Vuex states
   */
  lostRobot(state) {
    state.connectionState.robot = 'lost';
  },
  /**
   * Sets the robot connection state to failed.
   * @param {Vuex} state - Vuex states
   */
  failedToConnectToRobot(state) {
    state.connectionState.robot = 'failed';
  },
  /**
   * Sets the robot connection state to disconnected.
   * @param {Vuex} state - Vuex states
   */
  disconnectedFromRobot(state) {
    state.connectionState.robot = 'disconnected';
  },
  /**
   * Sets the UI App id.
   * @param {Vuex} state - Vuex states
   * @param {String} id - The id
   */
  setMyId(state, id) {
    state.myId = id;
  },
  /**
   * Clears the UI App id.
   * @param {Vuex} state - Vuex states
   */
  resetMyId(state) {
    state.myId = '';
  },
  /**
   * Sets the robot id.
   * @param {Vuex} state - Vuex states
   * @param {String} id - The robot id
   */
  setRobotId(state, id) {
    if (id) {
      state.robotId = id;
    }
  },
  /**
   * Clears the robot id.
   * @param {Vuex} state - Vuex states
   */
  resetRobotId(state) {
    if (state.robotId) {
      state.robotId = '';
    }
  },
  /**
   * Sets the camera stream.
   * @param {Vuex} state - Vuex states
   * @param {Object} stream - The camera stream object
   */
  setCameraStream(state, stream) {
    if (stream) {
      state.cameraStream = stream;
    }
  },
  /**
   * Clears the camera stream.
   * @param {Vuex} state - Vuex states
   */
  clearCameraStream(state) {
    if (state.cameraStream) {
      state.cameraStream = undefined;
    }
  },
  /**
   * Sets the map stream.
   * @param {Vuex} state - Vuex states
   * @param {Object} stream - The map stream object
   */
  setMapStream(state, stream) {
    if (stream) {
      state.mapStream = stream;
    }
  },
  /**
   * Clears the map stream.
   * @param {Vuex} state - Vuex states
   */
  clearMapStream(state) {
    if (state.mapStream) {
      state.mapStream = undefined;
    }
  },
  /**
   * Add robot to list.
   * @param {Vuex} state - Vuex states
   * @param {Object} robot - Robot to add.
   */
  addRobotToList(state, robot) {
    state.robotList.push(robot);
  },
  /**
   * Clears the robot list.
   * @param {Vuex} state - Vuex states
   */
  clearRobotList(state) {
    state.robotList = [];
  },
  /**
   * Sets the data channel availability to true.
   * @param {Vuex} state - Vuex states
   */
  enableDataChannel(state) {
    state.isDataChannelAvailable = true;
  },
  /**
   * Sets the data channel availability to false.
   * @param {Vuex} state - Vuex states
   */
  disableDataChannel(state) {
    state.isDataChannelAvailable = false;
  },
  /**
   * Sets the robot status.
   * @param {Vuex} state - Vuex states
   * @param {Object} status - The status
   */
  setRobotStatus(state, status) {
    state.robotStatus = status;
  },
  /**
   * Sets the patrol status.
   * @param {Vuex} state - Vuex states
   * @param {Object} status - The status
   */
  setPatrolStatus(state, status) {
    state.patrolStatus = status;
  },
};
