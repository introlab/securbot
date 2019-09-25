/* global easyrtc */
export default {
  connectToServer({ dispatch }) {
    console.log('Initializing...');
    easyrtc.setAutoInitUserMedia(false);
    easyrtc.enableVideo(false);
    easyrtc.enableAudio(false);
    easyrtc.enableVideoReceive(true);
    easyrtc.enableAudioReceive(false);
    easyrtc.enableDataChannels(true);

    easyrtc.setDataChannelOpenListener(id => dispatch('openedDataChannelListener', id));
    easyrtc.setDataChannelCloseListener(id => dispatch('closedDataChannelListener', id));
    easyrtc.setPeerListener((id, channel, data) => dispatch('handleData', { id, channel, data }));

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
    commit('resetMyId');
    easyrtc.hangupAll();
    easyrtc.disconnect();
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
  closedDataChannelListener({ state, commit, dispatch }, id) {
    commit('disableDataChannel');
    if (id === state.robotId || state.robotId) {
      commit('disconnectedFromRobot');
      commit('resetRobotId');
      dispatch('updateHTMLVideoElements', null, { root: true });
    }
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
  setStream({ state }, htmlElement) {
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
  },
  resetStream(_, htmlElement) {
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
