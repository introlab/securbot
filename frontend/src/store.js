import Vue from 'vue';
import Vuex from 'vuex';

Vue.use(Vuex);

/* global easyrtc */

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
    isDataChannelAvailable: false,
    joystickConfig: { // Joystick:absoluteMaxX | Joystick:absoluteMaxY
      maxX: 0,
      maxY: 0,
    },
    rates: {
      joystickCanvasRefreshRate: 1000 / 60, // Joystick:canvasRefreshRate
      joystickPositionRefreshRate: 100, // Joystick:operatorCommandInterval
      patrolCanvasRefreshRate: 1000 / 60, // PatrolMap:CanvasRefreshRate
    },
    htmlElement: {
      cameraId: 'camera-videobox-html-id',
      camera: null,
      mapId: 'map-videobox-html-id',
      map: null,
      patrolId: 'patrol-videobox-html-id',
      patrol: null,
    },
    patrol: {
      enable: false, // PatrolMap:enable
      patrolId: '', // PatrolMap:patrolMapId
      waypointList: [], // Patrol:waypointList | PatrolMap:waypointList
      // SaveLoad:patrolList | Patrol:patrolList
      patrolList: JSON.parse('[{"Name":"Test","waypoints":[{"x":593.2924107142857,"y":323.21428571428567,"yaw":0},{"x":550.4352678571429,"y":303.57142857142856,"yaw":0},{"x":518.2924107142858,"y":435.71428571428567,"yaw":0}]}]'),
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
    connected(state) {
      state.isConnected = true;
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
    setCameraHTMLElement(state, element) {
      state.htmlElement.camera = element;
    },
    clearCameraHTMLElement(state) {
      state.htmlElement.camera = null;
    },
    setMapHTMLElement(state, element) {
      state.htmlElement.map = element;
    },
    clearMapHTMLElement(state) {
      state.htmlElement.map = null;
    },
    setPatrolHTMLElement(state, element) {
      state.htmlElement.patrol = element;
    },
    clearPatrolHTMLElement(state) {
      state.htmlElement.patrol = null;
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
    enableDataChannel(state) {
      state.isDataChannelAvailable = true;
    },
    disableDataChannel(state) {
      state.isDataChannelAvailable = false;
    },
  },
  // Actions are methods that normally calls a mutation, but can do so asynchronously
  actions: {
    /* ***********************************EASYRTC**************************************** */
    connectToServer({ dispatch }) {
      console.log('Initializing...');
      easyrtc.setAutoInitUserMedia(false);
      easyrtc.enableVideo(false);
      easyrtc.enableAudio(false);
      easyrtc.enableVideoReceive(true);
      easyrtc.enableAudioReceive(false);
      easyrtc.enableDataChannels(true);

      // openedDataChannelListener({ state, commit, dispatch }, id)
      easyrtc.setDataChannelOpenListener(id => dispatch('openedDataChannelListener', id));
      // closedDataChannelListener({ state, commit, dispatch }, id)
      easyrtc.setDataChannelCloseListener(id => dispatch('closedDataChannelListener', id));
      // handleData({ state }, msg)
      easyrtc.setPeerListener((id, channel, data) => dispatch('handleData', { id, channel, data }));

      easyrtc.setRoomOccupantListener((roomName, occupants) => dispatch('handleRobotsInRoomNext', occupants));
      // acceptRobotVideo({ commit, dispatch }, robot)
      easyrtc.setStreamAcceptor((id, stream, streamName) => dispatch('acceptRobotVideo', { id, stream, streamName }));
      easyrtc.setOnStreamClosed(() => dispatch('closeRobotVideo'));
      // acceptCall(_, acceptor)
      easyrtc.setAcceptChecker((_, acceptor) => dispatch('acceptCall', acceptor));

      easyrtc.setRoomApiField('default', 'type', 'operator');

      // Uncomment next line to use the dev server
      easyrtc.setSocketUrl(process.env.VUE_APP_SERVER_URL);

      // This is the production line, only comment if necessary for debugging
      easyrtc.connect(
        process.env.VUE_APP_SERVER_ROOM_NAME,
        id => dispatch('loginSuccess', id), // this.loginSuccess
        (code, message) => dispatch('loginFailure', { code, message }), // this.loginFailure,
      );

      console.log('You are connected...');
    },
    handleRobotsInRoomNext({ commit }, occupants) {
      console.log(occupants);
      if (Object.keys(occupants).length) {
        for (const occupant in occupants) {
          if ('type' in occupants[occupant].apiField && occupants[occupant].apiField.type.fieldValue.includes('robot')) {
            const robot = {
              robotName: occupants[occupant].apiField.type.fieldValue,
              robotId: occupant,
            };
            commit('addRobotToList', robot);
          }
        }
      }
    },
    handleRobotsInRoom({ commit }, occupants) {
      commit('clearRobotList');
      console.log(occupants);
      for (const occupant in occupants) {
        if ('type' in occupants[occupant].apiField && occupants[occupant].apiField.type.fieldValue.includes('robot')) {
          const robot = {
            robotName: occupants[occupant].apiField.type.fieldValue,
            robotId: occupant,
          };
          commit('addRobotToList', robot);
        }
      }
    },
    connectToRobot({ commit, dispatch }, occupantId) {
      easyrtc.hangupAll();
      console.log(`Calling the chosen occupant : ${occupantId}`);

      commit('setRobotId', occupantId);
      commit('connectingToRobot');

      easyrtc.call(
        occupantId,
        (id, mediaType) => dispatch('callSuccessful', { id, mediaType }),
        (code, message) => dispatch('callFailure', { code, message }),
        (accepted, id) => dispatch('callAccepted', { accepted, id }),
      );
    },
    disconnectFromRobot({ commit }) {
      easyrtc.hangupAll();
      commit('disconnectedFromRobot');
      commit('resetRobotId');
    },
    callSuccessful({ commit }, robot) {
      console.warn(`Call to ${robot.id} was successful, here's the media: ${robot.mediaType}`);
      if (robot.mediaType === 'connection') {
        // this.teleopBus.$emit('connection-changed', 'connected');
        console.log('Connected!');
        commit('connectedToRobot');
      }
    },
    callFailure({ commit }, error) {
      console.warn(`Call failed : ${error.code} | ${error.message}`);
      commit('failedToConnectToRobot');
      commit('resetRobotId');
    },
    callAccepted({ commit }, result) {
      console.warn(`Call was ${result.accepted} from ${result.id}`);
      if (!result.accepted) {
        commit('failedToConnectToRobot');
        commit('resetRobotId');
      } else {
        commit('setRobotId', result.id);
      }
    },
    loginSuccess({ commit }, id) {
      console.warn(`I am ${easyrtc.idToName(id)}`);
      commit('connected');
      commit('setMyId', id);
    },
    loginFailure(_, error) {
      console.warn(`${error.code}:${error.message}`);
    },
    acceptRobotVideo({ commit, dispatch }, robot) {
      console.log(`Stream received info, id : ${robot.id}, streamName : ${robot.streamName}`);
      console.log(`checking incoming ${easyrtc.getNameOfRemoteStream(robot.id, robot.stream)}`);
      if (robot.streamName.includes('camera')) {
        commit('setCameraStream', robot.stream);
      } else if (robot.streamName.includes('map')) {
        commit('setMapStream', robot.stream);
      } else {
        console.warn('Unknown stream passed...');
      }
      dispatch('setHTMLVideoElements');
      // this.setHTMLVideoStream();
    },
    closeRobotVideo({ commit, dispatch }) {
      commit('clearCameraStream');
      commit('clearMapStream');
      dispatch('clearHTMLVideoElements');
      // this.clearHTMLVideoStream();
    },
    acceptCall(_, acceptor) {
      acceptor(false);
    },
    openedDataChannelListener({ state, commit, dispatch }, id) {
      console.warn(`Data channel open with ${id}`);
      // this.isDataChannelAvailable = true;
      commit('enableDataChannel');
      commit('enableJoystick');

      // Request the streams
      setTimeout(() => {
        if (!state.mapStream) {
          console.log('Requesting the map stream from peer...');
          dispatch('requestStreamFromRobot', 'map');
          // this.requestFeedFromPeer('map');
        }
        setTimeout(() => {
          if (!state.cameraStream) {
            console.log('Requesting the camera stream from peer...');
            dispatch('requestStreamFromRobot', 'camera');
            // this.requestFeedFromPeer('camera');
          }
        }, 1000);
      }, 1000);
    },
    closedDataChannelListener({ state, commit, dispatch }, id) {
      commit('disableDataChannel');
      if (id === state.robotId || state.robotId) {
        commit('disconnectedFromRobot');
        commit('resetRobotId');

        // this.teleopBus.$emit('connection-changed', 'disconnected');
        dispatch('clearHTMLVideoElements');
        // this.clearHTMLVideoStream();
      }
      commit('disableJoystick');
      // this.joystickState = 'disable';
      // this.teleopBus.$emit('on-joystick-state-changed', this.joystickState);
    },
    sendJoystickPosition({ state, dispatch }, data) {
      dispatch('sendData', { id: state.robotId, channel: 'joystick-position', data: JSON.stringify(data) });
    },
    sendPatrol({ dispatch }, data) {
      dispatch('sendData', { id: this.state.robotId, channel: 'patrol-plan', data: JSON.stringify(data) });
    },
    requestStreamFromRobot({ state, dispatch }, feed) {
      dispatch('sendData', { id: state.robotId, channel: 'onDemandStream', data: feed });
    },
    sendData({ state }, msg) {
      if (state.isDataChannelAvailable && state.robotId) {
        easyrtc.sendDataP2P(msg.id, msg.channel, msg.data);
      } else {
        console.warn('No data channel or peer available to send data...');
      }
    },
    handleData({ state }, msg) {
      if (state.robotId === msg.id) {
        console.log(`Received data from ${msg.id} on channel ${msg.channel}:`);
        console.log(msg.data);
      } else {
        console.log('Received data from someone else than the peer, ignoring it...');
      }
    },
    setHTMLVideoElements({ state }) {
      console.log('Setting html elements...');
      if (state.htmlElement.camera && state.cameraStream) {
        console.log('Setting camera stream...');
        easyrtc.setVideoObjectSrc(state.htmlElement.camera, state.cameraStream);
      }
      if (state.htmlElement.map && state.mapStream) {
        console.log('Setting map stream...');
        easyrtc.setVideoObjectSrc(state.htmlElement.map, state.mapStream);
      }
      if (state.htmlElement.patrol && state.mapStream) {
        console.log('Setting patrol stream...');
        easyrtc.setVideoObjectSrc(state.htmlElement.patrol, state.mapStream);
      }
    },
    clearHTMLVideoElements({ state }) {
      if (state.cameraStreamElement) {
        easyrtc.setVideoObjectSrc(state.cameraStreamElement, '');
      }
      if (state.mapStreamElement) {
        easyrtc.setVideoObjectSrc(state.mapStreamElement, '');
      }
      if (state.patrolMapStreamElement) {
        easyrtc.setVideoObjectSrc(state.patrolMapStreamElement, '');
      }
    },
    getPatrolList() {
    },
  },
});
