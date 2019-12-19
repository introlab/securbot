/* global easyrtc */
/**
 * The EasyRTC-Client module actions.
 *
 * @module EasyRTC-Client
 * @exports
 */
export default {
  /**
   * Connects to the server and sets the callbacks.
   * @param {Vuex} Vuex
   */
  connectToServer({ commit, dispatch }) {
    commit('setServerConnState', 'connecting');
    easyrtc.setAutoInitUserMedia(false);
    easyrtc.enableVideo(false);
    easyrtc.enableAudio(false);
    easyrtc.enableVideoReceive(true);
    easyrtc.enableAudioReceive(false);
    easyrtc.enableDataChannels(true);

    easyrtc.setDataChannelOpenListener(id => dispatch('openedDataChannelListener', id));
    easyrtc.setDataChannelCloseListener(() => commit('disableDataChannel'));
    easyrtc.setPeerListener((id, channel, data) => commit('setMapSize', data, { root: true }), 'map-size');
    easyrtc.setPeerListener((id, channel, data) => dispatch('handleRobotStatus', { id, channel, data }), 'robot-status');
    easyrtc.setPeerListener((id, channel, data) => dispatch('handlePatrolStatus', { id, channel, data }), 'patrol-status');
    easyrtc.setPeerListener((id, channel, data) => dispatch('handleData', { id, channel, data }));
    easyrtc.setPeerClosedListener((id, other) => dispatch('handleRobotDisconnection', { id, other }));

    easyrtc.setRoomOccupantListener((roomName, occupants) => dispatch('handleRobotsInRoomNext', occupants));
    easyrtc.setStreamAcceptor((id, stream, streamName) => dispatch('acceptRobotVideo', { id, stream, streamName }));
    easyrtc.setOnStreamClosed(() => dispatch('closeRobotVideo'));
    easyrtc.setAcceptChecker((_, acceptor) => dispatch('acceptCall', acceptor));

    easyrtc.setRoomApiField('default', 'type', 'operator');

    easyrtc.setSocketUrl(process.env.VUE_APP_SERVER_URL);

    easyrtc.connect(
      process.env.VUE_APP_SERVER_ROOM_NAME,
      id => dispatch('loginSuccess', id),
      (code, message) => dispatch('loginFailure', { code, message }),
    );
  },
  /**
   * Disconnect from the server.
   * @param {Vuex} Vuex
   */
  disconnectFromServer({ commit }) {
    commit('setServerConnState', 'disconnected');
    commit('resetMyId');
    easyrtc.hangupAll();
    easyrtc.disconnect();
  },
  /**
   * Handles the robots in the room by extracting their api fields.
   * @param {Vuex} Vuex
   * @param {Array} occupants - The occupants of the room
   */
  handleRobotsInRoomNext({ commit }, occupants) {
    commit('clearRobotList');
    if (Object.keys(occupants).length) {
      for (const occupant in occupants) {
        if ('type' in occupants[occupant].apiField && occupants[occupant].apiField.type.fieldValue === 'robot') {
          const robot = {
            robotName: occupants[occupant].apiField.name.fieldValue,
            robotId: occupant,
          };
          commit('addRobotToList', robot);
        }
      }
    }
  },
  /**
   * Connects to a robot.
   * @param {Vuex} Vuex
   * @param {String} occupantId - The robot id to call.
   */
  connectToRobot({ commit, dispatch }, occupantId) {
    easyrtc.hangupAll();

    commit('setRobotId', occupantId);
    commit('connectingToRobot');

    easyrtc.call(
      occupantId,
      (id, mediaType) => dispatch('callSuccessful', { id, mediaType }),
      (code, message) => dispatch('callFailure', { code, message }),
      (accepted, id) => dispatch('callAccepted', { accepted, id }),
    );
  },
  /**
   * Disconnects from the current connected robot.
   * @param {Vuex} Vuex
   */
  disconnectFromRobot({ commit }) {
    easyrtc.hangupAll();
    commit('disconnectedFromRobot');
    commit('resetRobotId');
  },
  /**
   * Callbacks for a robot disconnection.
   * @param {Vuex} Vuex
   * @param {String} id - The id of the disconnected robot.
   */
  handleRobotDisconnection({ state, commit, dispatch }, id) {
    if (id === state.robotId) {
      commit('disconnectedFromRobot');
      commit('resetRobotId');
      dispatch('updateHTMLVideoElements', null, { root: true });
    }
  },
  /**
   * Call successful callback.
   * @param {Vuex} Vuex
   * @param {Object} robot - The robot media object
   */
  callSuccessful({ commit }, robot) {
    if (robot.mediaType === 'connection') {
      commit('connectedToRobot');
    }
  },
  /**
   * Call failed callback.
   * @param {Vuex} Vuex
   */
  callFailure({ commit }) {
    commit('failedToConnectToRobot');
    commit('resetRobotId');
  },
  /**
   * Call accepted callback.
   * @param {Vuex} Vuex
   * @param {Object} result - The result of the call
   */
  callAccepted({ commit }, result) {
    if (!result.accepted) {
      commit('failedToConnectToRobot');
      commit('resetRobotId');
    } else {
      commit('setRobotId', result.id);
    }
  },
  /**
   * Login success callback.
   * @param {Vuex} Vuex
   * @param {String} id - The id given to the UI by the server
   */
  loginSuccess({ commit }, id) {
    console.warn(`I am ${easyrtc.idToName(id)}`);
    commit('setServerConnState', 'connected');
    // commit('connected');
    commit('setMyId', id);
  },
  /**
   * Login failure callback.
   * @param {Vuex} Vuex
   * @param {Error} error - The error object
   */
  loginFailure({ commit }, error) {
    commit('setServerConnState', 'failed');
    console.warn(`${error.code}:${error.message}`);
  },
  /**
   * Accepts and sets a video stream from the robot.
   * @param {Vuex} Vuex
   * @param {Object} robot - The robot object
   */
  acceptRobotVideo({ commit, dispatch }, robot) {
    if (robot.streamName.includes('camera')) {
      commit('setCameraStream', robot.stream);
    } else if (robot.streamName.includes('map')) {
      commit('setMapStream', robot.stream);
    } else {
      console.warn('Unknown stream passed...');
    }
    dispatch('updateHTMLVideoElements', null, { root: true });
  },
  /**
   * Closes the robot video stream (clears).
   * @param {Vuex} Vuex
   */
  closeRobotVideo({ commit, dispatch }) {
    commit('clearCameraStream');
    commit('clearMapStream');
    dispatch('updateHTMLVideoElements', null, { root: true });
  },
  /**
   * Accept call callback, as an operator, the call is always refused.
   * @param {Vuex} _
   * @param {Function} acceptor - The acceptor function
   */
  acceptCall(_, acceptor) {
    acceptor(false);
  },
  /**
   * Data channel 'open' callback.
   * @param {Vuex} Vuex
   */
  openedDataChannelListener({ state, commit, dispatch }) {
    commit('enableDataChannel');

    // Request the streams
    setTimeout(() => {
      if (!state.mapStream) {
        dispatch('requestStreamFromRobot', 'map');
      }
      setTimeout(() => {
        if (!state.cameraStream) {
          dispatch('requestStreamFromRobot', 'camera');
        }
      }, 1000);
    }, 1000);
  },
  /**
   * Requests a stream/feed from the connected robot through the data channel.
   * @param {Vuex} Vuex
   * @param {String} feed - The name of the stream/feed
   */
  requestStreamFromRobot({ state, dispatch }, feed) {
    dispatch('sendData', { id: state.robotId, channel: 'onDemandStream', data: feed });
  },
  /**
   * Sends data to robot.
   * @param {Vuex} Vuex
   * @param {any} msg - The data/msg to send
   */
  sendData({ state }, msg) {
    if (state.isDataChannelAvailable && state.robotId) {
      easyrtc.sendDataP2P(msg.id, msg.channel, msg.data);
    } else {
      console.warn('No data channel or peer available to send data...');
    }
  },
  /**
   * Handles icoming data.
   * @param {Vuex} Vuex
   * @param {String} msg - The incoming data
   */
  handleData({ state }, msg) {
    if (state.robotId === msg.id) {
      console.log(`Received data from ${msg.id} on channel ${msg.channel}:`);
      console.log(msg.data);
    } else {
      console.log('Received data from someone else than the peer, ignoring it...');
    }
  },
  /**
   * Handles the robot status icoming messages.
   * @param {Vuex} Vuex
   * @param {String} msg - The incoming status data
   */
  handleRobotStatus({ state, commit }, msg) {
    if (state.robotId === msg.id) {
      const status = JSON.parse(msg.data);
      commit('setRobotStatus', status);
    }
  },
  /**
   * Handles the patrol status icoming messages.
   * @param {Vuex} Vuex
   * @param {String} msg - The incoming status data
   */
  handlePatrolStatus({ state, commit }, msg) {
    if (state.robotId === msg.id) {
      const data = JSON.parse(msg.data);
      const status = {
        state: data.status,
        planned: data.goalsPlanned,
        reached: data.goalsReached,
      };
      commit('setPatrolStatus', status);
    }
  },
  /**
   * Sets the video stream to the HTML video elements of the UI.
   * @param {Vuex} Vuex
   * @param {HTMLVideoElement} htmlElement - The HTML video element to set
   */
  setStreams({ state }, htmlElement) {
    if (htmlElement.camera && state.cameraStream) {
      easyrtc.setVideoObjectSrc(htmlElement.camera, state.cameraStream);
    }
    if (htmlElement.map && state.mapStream) {
      easyrtc.setVideoObjectSrc(htmlElement.map, state.mapStream);
    }
    if (htmlElement.patrol && state.mapStream) {
      easyrtc.setVideoObjectSrc(htmlElement.patrol, state.mapStream);
    }
    if (htmlElement.event && state.mapStream) {
      easyrtc.setVideoObjectSrc(htmlElement.event, state.mapStream);
    }
  },
  /**
   * Resets the HTML video elements.
   * @param {Vuex} _
   * @param {HTMLVideoElement} htmlElement - The HTML video element to clear
   */
  resetStreams(_, htmlElement) {
    if (htmlElement.camera) {
      easyrtc.setVideoObjectSrc(htmlElement.camera, '');
    }
    if (htmlElement.map) {
      easyrtc.setVideoObjectSrc(htmlElement.map, '');
    }
    if (htmlElement.patrol) {
      easyrtc.setVideoObjectSrc(htmlElement.patrol, '');
    }
  },
};
