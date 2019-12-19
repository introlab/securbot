/**
 * The EasyRTC-Client module states.
 *
 * @module EasyRTC-Client
 * @exports
 */
export default {
  /**
   * The connection state between the UI app and the server/robot.
   */
  connectionState: {
    server: 'disconnected',
    robot: 'disconnected',
  },
  /**
   * The id of the UI app.
   */
  myId: '',
  /**
   * The id of the connected robot.
   */
  robotId: '',
  /**
   * The camera stream object.
   */
  cameraStream: undefined,
  /**
   * The map stream object.
   */
  mapStream: undefined,
  /**
   * List of all robot in the room (EasyRTC).
   */
  robotList: [],
  /**
   * Is their a data channel open with the robot.
   */
  isDataChannelAvailable: false,
  /**
   * The robot status.
   */
  robotStatus: {
    resources: {
      cpu: 0,
      mem: 0,
    },
    wifi: {
      rssi: 0,
      noise: 0,
    },
    intention: {
      strategies: [],
      desires: [],
    },
  },
  /**
   * The patrol status of the robot.
   */
  patrolStatus: {
    state: 'None',
    planned: 0,
    reached: 0,
  },
};
