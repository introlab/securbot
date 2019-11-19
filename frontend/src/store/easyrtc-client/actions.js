/* global easyrtc */
export default {
  testLogger(_, data) {
    console.log('Test Logger:');
    console.log(data);
  },
  connectToServer({ commit, dispatch }) {
    console.log('Initializing...');
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

    console.log('You are connected...');
  },
  disconnectFromServer({ commit }) {
    commit('setServerConnState', 'disconnected');
    commit('resetMyId');
    easyrtc.hangupAll();
    easyrtc.disconnect();
  },
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
  handleRobotDisconnection({ state, commit, dispatch }, id) {
    if (id === state.robotId) {
      commit('disconnectedFromRobot');
      commit('resetRobotId');
      dispatch('updateHTMLVideoElements', null, { root: true });
    }
  },
  callSuccessful({ commit }, robot) {
    console.warn(`Call to ${robot.id} was successful, here's the media: ${robot.mediaType}`);
    if (robot.mediaType === 'connection') {
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
    commit('setServerConnState', 'connected');
    // commit('connected');
    commit('setMyId', id);
  },
  loginFailure({ commit }, error) {
    commit('setServerConnState', 'failed');
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
    dispatch('updateHTMLVideoElements', null, { root: true });
  },
  closeRobotVideo({ commit, dispatch }) {
    commit('clearCameraStream');
    commit('clearMapStream');
    dispatch('updateHTMLVideoElements', null, { root: true });
  },
  acceptCall(_, acceptor) {
    acceptor(false);
  },
  openedDataChannelListener({ state, commit, dispatch }, id) {
    console.warn(`Data channel open with ${id}`);
    commit('enableDataChannel');

    // Request the streams
    setTimeout(() => {
      if (!state.mapStream) {
        console.log('Requesting the map stream from peer...');
        dispatch('requestStreamFromRobot', 'map');
      }
      setTimeout(() => {
        if (!state.cameraStream) {
          console.log('Requesting the camera stream from peer...');
          dispatch('requestStreamFromRobot', 'camera');
        }
      }, 1000);
    }, 1000);
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
  handleRobotStatus({ state, commit }, msg) {
    if (state.robotId === msg.id) {
      const status = JSON.parse(msg.data);
      console.log(`Got status ${status}`);
      commit('setRobotStatus', status);
    }
  },
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
  setStreams({ state }, htmlElement) {
    console.log('Setting html elements...');
    if (htmlElement.camera && state.cameraStream) {
      console.log('Setting camera stream...');
      easyrtc.setVideoObjectSrc(htmlElement.camera, state.cameraStream);
    }
    if (htmlElement.map && state.mapStream) {
      console.log('Setting map stream...');
      easyrtc.setVideoObjectSrc(htmlElement.map, state.mapStream);
    }
    if (htmlElement.patrol && state.mapStream) {
      console.log('Setting patrol stream...');
      easyrtc.setVideoObjectSrc(htmlElement.patrol, state.mapStream);
    }
    if (htmlElement.event && state.mapStream) {
      console.log('Setting patrol stream...');
      easyrtc.setVideoObjectSrc(htmlElement.event, state.mapStream);
    }
  },
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
