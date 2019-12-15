export default {
  sendJoystickPosition({ state, dispatch }, data) {
    dispatch('client/sendData', { id: state.client.robotId, channel: 'joystick-position', data: JSON.stringify(data) });
  },
  sendPatrol({ state, dispatch }, data) {
    dispatch('client/sendData', { id: state.client.robotId, channel: 'patrol-plan', data: JSON.stringify(data) });
  },
  sendGoTo({ state, dispatch }, data) {
    dispatch('client/sendData', { id: state.client.robotId, channel: 'goto', data: JSON.stringify(data) });
  },
  sendDockingProcess({ state, dispatch }, data) {
    dispatch('client/sendData', { id: state.client.robotId, channel: 'force-dock', data: JSON.stringify(data) });
  },
  startDockingProcess({ commit, dispatch }) {
    commit('setDockingProcess', setInterval(() => { dispatch('sendDockingProcess', { force: true }); }, 1000));
  },
  stopDockingProcess({ commit, dispatch }) {
    commit('clearDockingProcess');
    dispatch('sendDockingProcess', { force: false });
  },
  sendChangeMapZoom({ state, dispatch }, data) {
    dispatch('client/sendData', { id: state.client.robotId, channel: 'changeMapZoom', data });
  },
  setHTMLVideoElements({ state, dispatch }) {
    dispatch('client/setStreams', state.htmlElement);
  },
  getHTMLVideoElements({ state, commit }) {
    commit('setCameraHTMLElement', document.getElementById(state.htmlElement.cameraId));
    commit('setMapHTMLElement', document.getElementById(state.htmlElement.mapId));
    commit('setPatrolHTMLElement', document.getElementById(state.htmlElement.patrolId));
    commit('setEventHTMLElement', document.getElementById(state.htmlElement.eventId));
  },
  updateHTMLVideoElements({ dispatch }) {
    dispatch('clearHTMLVideoElements');
    dispatch('getHTMLVideoElements');
    dispatch('setHTMLVideoElements');
  },
  clearHTMLVideoElements({ state, commit, dispatch }) {
    dispatch('client/resetStreams', state.htmlElement);
    commit('clearCameraHTMLElement');
    commit('clearMapHTMLElement');
    commit('clearPatrolHTMLElement');
    commit('clearEventHTMLElement');
  },
  stopTeleop({ state, dispatch }) {
    const end = {
      x: 0,
      y: 0,
      enabled: false,
    };
    dispatch('client/sendData', { id: state.client.robotId, channel: 'joystick-position', data: JSON.stringify(end) });
  },
  getPatrols() {
  },
  savePatrols() {
  },
};
