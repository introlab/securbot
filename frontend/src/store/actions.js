/**
 * The root store actions.
 *
 * @module Store
 * @exports
 */
export default {
  /**
   * Sends data on datachannel: 'joystick-position'.
   * @param {Vuex} Vuex
   * @param {any} data - A value to send on the datachannel, will be stringify
   */
  sendJoystickPosition({ state, dispatch }, data) {
    dispatch('client/sendData', { id: state.client.robotId, channel: 'joystick-position', data: JSON.stringify(data) });
  },
  /**
   * Sends data on datachannel: 'patrol-plan'.
   * @param {Vuex} Vuex
   * @param {any} data - A value to send on the datachannel, will be stringify
   */
  sendPatrol({ state, dispatch }, data) {
    dispatch('client/sendData', { id: state.client.robotId, channel: 'patrol-plan', data: JSON.stringify(data) });
  },
  /**
   * Sends data on datachannel: 'goto'.
   * @param {Vuex} Vuex
   * @param {any} data - A value to send on the datachannel, will be stringify
   */
  sendGoTo({ state, dispatch }, data) {
    dispatch('client/sendData', { id: state.client.robotId, channel: 'goto', data: JSON.stringify(data) });
  },
  /**
   * Sends data on datachannel: 'force-dock'.
   * @param {Vuex} Vuex
   * @param {any} data - A value to send on the datachannel, will be stringify
   */
  sendDockingProcess({ state, dispatch }, data) {
    dispatch('client/sendData', { id: state.client.robotId, channel: 'force-dock', data: JSON.stringify(data) });
  },
  /**
   * Starts the docking process.
   * @param {Vuex} Vuex
   */
  startDockingProcess({ commit, dispatch }) {
    commit('setDockingProcess', setInterval(() => { dispatch('sendDockingProcess', { force: true }); }, 1000));
  },
  /**
   * Sends data for docking'.
   * @param {Vuex} Vuex
   */
  stopDockingProcess({ commit, dispatch }) {
    commit('clearDockingProcess');
    dispatch('sendDockingProcess', { force: false });
  },
  /**
   * Sends data on datachannel: 'changeMapZoom'.
   * @param {Vuex} Vuex
   * @param {any} data - A value to send on the datachannel, will be stringify
   */
  sendChangeMapZoom({ state, dispatch }, data) {
    dispatch('client/sendData', { id: state.client.robotId, channel: 'changeMapZoom', data });
  },
  /**
   * Sets the html video elements.
   * @param {Vuex} Vuex
   */
  setHTMLVideoElements({ state, dispatch }) {
    dispatch('client/setStreams', state.htmlElement);
  },
  /**
   * Gets the html video element.
   * @param {Vuex} Vuex
   */
  getHTMLVideoElements({ state, commit }) {
    commit('setCameraHTMLElement', document.getElementById(state.htmlElement.cameraId));
    commit('setMapHTMLElement', document.getElementById(state.htmlElement.mapId));
    commit('setPatrolHTMLElement', document.getElementById(state.htmlElement.patrolId));
    commit('setEventHTMLElement', document.getElementById(state.htmlElement.eventId));
  },
  /**
   * Updates the html video elements with the available stream.
   * @param {Vuex} Vuex
   */
  updateHTMLVideoElements({ dispatch }) {
    dispatch('clearHTMLVideoElements');
    dispatch('getHTMLVideoElements');
    dispatch('setHTMLVideoElements');
  },
  /**
   * Clears all binding to the html video element.
   * @param {Vuex} Vuex
   */
  clearHTMLVideoElements({ state, commit, dispatch }) {
    dispatch('client/resetStreams', state.htmlElement);
    commit('clearCameraHTMLElement');
    commit('clearMapHTMLElement');
    commit('clearPatrolHTMLElement');
    commit('clearEventHTMLElement');
  },
  /**
   * Sends a last command for teleop to make the robot aware that there is no more remote command.
   * @param {Vuex} Vuex
   */
  stopTeleop({ state, dispatch }) {
    const end = {
      x: 0,
      y: 0,
      enabled: false,
    };
    dispatch('client/sendData', { id: state.client.robotId, channel: 'joystick-position', data: JSON.stringify(end) });
  },
};
