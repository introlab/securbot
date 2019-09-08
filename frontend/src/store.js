import Vue from 'vue';
import Vuex from 'vuex';

Vue.use(Vuex);

export default new Vuex.Store({
  // State = var/let calls
  state: {
    // /////////////////////////////////// //
    test: {
      OwO: {
        apiField: {
          type: {
            fieldValue: 'robot-OwO',
          },
        },
      },
      UwU: {
        apiField: {
          type: {
            fieldValue: 'robot-UwU',
          },
        },
      },
    },
    // //////////////////////////////////// //
    theme: {
      white: {

      },
      dark: {

      },
    },
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
    showStreams: true, // VideoBox:show
    joystickEnabled: false, // Layout:joystickState | Joystick:enable | Teleop:enableJoystick
    connecting: false, // Connection:waitingForConnectionState
    joystickConfig: { // Joystick:absoluteMaxX | Joystick:absoluteMaxY
      maxX: 0,
      maxY: 0,
    },
    intervals: {
      joystickCanvasRefreshRate: 1000 / 60, // Joystick:canvasRefreshRate
      joystickPositionRefreshRate: 100, // Joystick:operatorCommandInterval
      patrolCanvasRefreshRate: 1000 / 60, // PatrolMap:CanvasRefreshRate
    },
    patrol: {
      enable: false, // PatrolMap:enable
      patrolId: '', // PatrolMap:patrolMapId
      waypointList: [], // Patrol:waypointList | PatrolMap:waypointList
      patrolList: [], // SaveLoad:patrolList | Patrol:patrolList
    },
  },
  // Mutation is the same thing as a set call, needs to be synchronous
  mutations: {
    /**
     * Updates the server connection state.
     * * Connected
     * * Connecting
     * * Disconnected
     * * Error
     *
     * @param {*} state
     * @param {*} serverState
     */
    setServerConnState(state, serverState) {
      state.connectionState.server = serverState;
    },
    /**
     * Updates the robot/peer connection state.
     * * Connected
     * * Connecting
     * * Disconnected
     * * Error
     *
     * @param {*} state
     * @param {*} robotState
     */
    connectingToRobot(state, robotState) {
      state.connectionState.robot = robotState;
    },
    /**
     * Shows the stream of the robot.
     *
     * @param {*} state
     */
    showStreams(state) {
      state.showStreams = true;
    },
    /**
     * Hides the stream of the robot.
     *
     * @param {*} state
     */
    hideStreams(state) {
      state.showStreams = false;
    },
    /**
     * Sets the client ID.
     *
     * @param {*} state
     * @param {*} id
     */
    setMyId(state, id) {
      state.myId = id;
    },
    /**
     * Resets the client ID.
     *
     * @param {*} state
     */
    resetMyId(state) {
      state.myId = '';
    },
    /**
     * Sets the robot ID to the one connected to.
     *
     * @param {*} state
     * @param {*} id
     */
    setRobotId(state, id) {
      if (id) {
        state.robotId = id;
      }
    },
    /**
     * Resets the robot ID.
     *
     * @param {*} state
     */
    resetRobotId(state) {
      if (state.robotId) {
        state.robotId = '';
      }
    },
    /**
     * Enables the joystick ability to send data.
     *
     * @param {*} state
     */
    enableJoystick(state) {
      state.joystickEnabled = true;
    },
    /**
     * Disables the joystick ability to send data.
     *
     * @param {*} state
     */
    disableJoystick(state) {
      state.joystickEnabled = false;
    },
    /**
     * Sets the camera stream.
     *
     * @param {*} state
     * @param {*} stream
     */
    setCameraStream(state, stream) {
      if (stream) {
        state.cameraStream = stream;
      }
    },
    /**
     * Clears the camera stream.
     *
     * @param {*} state
     */
    clearCameraStream(state) {
      if (state.cameraStream) {
        state.cameraStream = undefined;
      }
    },
    /**
     * Sets the map stream.
     *
     * @param {*} state
     * @param {*} stream
     */
    setMapStream(state, stream) {
      if (stream) {
        state.mapStream = stream;
      }
    },
    /**
     * Clears the map stream.
     *
     * @param {*} state
     */
    clearMapStream(state) {
      if (state.mapStream) {
        state.mapStream = undefined;
      }
    },
    /**
     * Adds a waypoint to the list. If the given object have an index prop, add the element to that
     * index, else add it at the end.
     *
     * @param {*} state
     * @param {*} add
     */
    addWaypoint(state, add) {
      if ('index' in add) {
        state.patrol.waypointList.splice(add.index, 0, add.wp);
      } else {
        state.patrol.waypointList.push(add.wp);
      }
    },
    /**
     * Updates a waypoint in the list. Expects an object with an index and wp props.
     *
     * @param {*} state
     * @param {*} update
     */
    updateWaypoint(state, update) {
      state.patrol.waypointList.splice(update.index, 1, update.wp);
    },
    /**
     * Removes the wp at the given index from the list.
     *
     * @param {*} state
     * @param {number} index
     */
    removeWaypoint(state, index) {
      state.patrol.waypointList.splice(index, 1);
    },
    /**
     * Clears the entire waypoint list.
     *
     * @param {*} state
     */
    clearWaypointList(state) {
      state.patrol.waypointList = [];
    },
    /**
     * Fills/Replaces the waypoint list with the one given.
     *
     * @param {*} state
     * @param {*} waypointList
     */
    fillWaypointList(state, waypointList) {
      state.patrol.waypointList = waypointList;
    },
    /**
     * Adds a patrol to the list.
     *
     * @param {*} state
     * @param {*} patrol
     */
    addPatrol(state, patrol) {
      state.patrol.patrolList.push(patrol);
    },
    /**
     * Updates a patrol in the list. Expects an object with an index and patrol props.
     *
     * @param {*} state
     * @param {*} update
     */
    updatePatrol(state, update) {
      state.patrol.waypointList.splice(update.index, 1, update.patrol);
    },
    /**
     * Removes the patrol at the given index from the list.
     *
     * @param {*} state
     * @param {*} index
     */
    removePatrol(state, index) {
      state.patrol.patrolList.splice(index, 1);
    },
    /**
     * Clears the entire patrol list.
     *
     * @param {*} state
     */
    clearPatrol(state) {
      state.patrol.patrolList = [];
    },
    clearRobotList(state) {
      state.robotList = [];
    },
    addRobotToList(state, robot) {
      state.robotList.push(robot);
    },
  },
  // Actions are methods that normally calls a mutation, but can do so asynchronously
  actions: {
    /**
     * Sends the joystick position to the connected robot.
     *
     */
    sendJoystickPosition() {
    },
    /**
     * Gets the patrol list from the database.
     *
     */
    getPatrolList() {
    },
    /**
     * Sends a patrol to the database to be saved.
     *
     */
    sendPatrol() {
    },
    /**
     * Connects to the easyrtc server.
     *
     */
    connectToServer() {
    },
    /**
     * Connects to the given robot id.
     *
     */
    connectToRobot() {
    },
    handleRobotsInRoom({ commit }, occupants) {
      commit('clearRobotList');
      console.log(occupants);
      for (const occupant in occupants) {
        if (occupants[occupant].apiField.type.fieldValue.includes('robot')) {
          const robot = {
            robotName: occupants[occupant].apiField.type.fieldValue,
            robotId: occupant,
          };
          commit('addRobotToList', robot);
        }
      }
    },
  },
});
