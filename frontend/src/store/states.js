export default {
  darkMode: true,
  showStreams: true,
  currentRobot: {
    name: '',
    id: {
      client: '',
      db: '',
    },
  },
  mapZoom: 1,
  mapSize: {
    width: 2400,
    height: 2400,
  },
  joystickEnabled: false,
  joystickConfig: {
    maxX: 0,
    maxY: 0,
  },
  rates: {
    joystickCanvasRefreshRate: 60,
    joystickPositionRefreshRate: 100,
    patrolCanvasRefreshRate: 60,
  },
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
  patrol: {
    enable: false,
    waypointList: [],
    waypointHeaders: [{ key: 'index', label: '#' }, { key: 'x', label: 'X', formatter: 'fixFloat' }, { key: 'y', label: 'Y', formatter: 'fixFloat' }, { key: 'yaw', label: 'Yaw', formatter: 'fixFloat' }, { key: 'remove', label: 'Remove' }],
    patrolList: [],
    scheduleList: [],
  },
};
