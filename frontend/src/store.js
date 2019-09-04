import Vue from 'vue';
import Vuex from 'vuex';

Vue.use(Vuex);

export default new Vuex.Store({
  // State = var/let calls
  state: {
    isConnected: false, // Connection:isConnected
    myId: undefined, // Layout:selfEasyrtcid
    robotId: undefined, // Layout:peerId
    cameraStream: undefined, // Layout:cameraStream
    mapStream: undefined, // Layout:mapStream | Teleop:showCamera|showMap
    robotIdTable: [], // Layout:peerTable
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

  },
  // Actions are methods that normally calls a mutation, but can do so asynchronously
  actions: {

  },
});
