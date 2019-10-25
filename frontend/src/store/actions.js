export default {
  sendJoystickPosition({ state, dispatch }, data) {
    dispatch('client/sendData', { id: state.client.robotId, channel: 'joystick-position', data: JSON.stringify(data) });
  },
  sendPatrol({ state, dispatch }, data) {
    dispatch('client/sendData', { id: state.client.robotId, channel: 'patrol-plan', data: JSON.stringify(data) });
  },
  setHTMLVideoElements({ state, dispatch }) {
    dispatch('client/setStreams', state.htmlElement);
  },
  getHTMLVideoElements({ state, commit }) {
    commit('setCameraHTMLElement', document.getElementById(state.htmlElement.cameraId));
    commit('setMapHTMLElement', document.getElementById(state.htmlElement.mapId));
    commit('setPatrolHTMLElement', document.getElementById(state.htmlElement.patrolId));
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
  },
  getPatrols() {
  },
  savePatrols() {
  },
};