export default {
  darkMode: true,
  showStreams: true,
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
    waypointHeaders: ['X', 'Y', 'Yaw'],
    patrolList: JSON.parse('[{"Name":"Test","waypoints":[{"x":593.2924107142857,"y":323.21428571428567,"yaw":0},{"x":550.4352678571429,"y":303.57142857142856,"yaw":0},{"x":518.2924107142858,"y":435.71428571428567,"yaw":0}]}]'),
  },
};
