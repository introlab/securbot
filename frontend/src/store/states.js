/**
 * The root store states.
 *
 * @module Store
 * @exports
 */
export default {
  /**
   * The robot object of the current connected robot. If not connected to a robot, all properties
   * are going to be undefined.
   */
  currentRobot: {
    name: '', // The robot name
    id: {
      client: '', // The easyrtc robot id
      db: '', // The database robot id
    },
  },
  /**
   * The value of the zoom tranform for the map stream
   */
  mapZoom: 1,
  /**
   * The map size to compute absolute waypoint coordinates
   */
  mapSize: {
    width: 2400,
    height: 2400,
  },
  /**
   * The state of the joystick
   */
  joystickEnabled: false,
  /**
   * The docking setInterval object
   */
  dockingInterval: '',
  /**
   * The different update rates of some interactive components
   */
  rates: {
    joystickCanvasRefreshRate: 60,
    joystickPositionRefreshRate: 100,
    patrolCanvasRefreshRate: 60,
  },
  /**
   * An object that keep tracks of html elements for video streams binding
   */
  htmlElement: {
    cameraId: 'camera-videobox-html-id',
    camera: null,
    mapId: 'map-videobox-html-id',
    map: null,
    patrolId: 'patrol-videobox-html-id',
    patrol: null,
    eventId: 'event-videobox-html-id',
    event: null,
  },
  /**
   * The headers use for the different vue-bootstrap table
   */
  headers: {
    waypoints: [
      { key: 'index', label: '#' },
      { key: 'label', label: 'Label' },
      { key: 'holdTime', label: 'Hold Time' },
      { key: 'remove', label: 'Remove' },
    ],
    events: [
      { key: 'time', label: 'DateTime', sortable: true },
      { key: 'robot', label: 'Robot', sortable: true },
      { key: 'object', label: 'Object', sortable: true },
      { key: 'context', label: 'Context' },
      { key: 'description_text', label: 'Description' },
      { key: 'tags', label: 'Tags' },
      { key: 'image', label: 'Image' },
    ],
  },
  /**
   * A patrol object that keep track of all patrols found on the database and the current patrol
   * being configured by the user.
   */
  patrol: {
    current: {
      id: '',
      obj: {
        name: '',
        robot: '',
        description_text: '',
        last_modified: '',
        waypoints: [],
      },
    },
    list: [],
  },
  /**
   * A schedule object that keep track of all patrols found on the database and the current schedule
   * being configured by the user.
   */
  schedule: {
    current: {
      id: '',
      obj: {
        name: '',
        robot: '',
        description_text: '',
        patrol: '',
        last_modified: '',
        cron: '',
        timeout_s: '',
        repetitions: '',
        enabled: true,
      },
    },
    list: [],
  },
};
