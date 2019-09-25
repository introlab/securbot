export default {
  darkMode: true,
  showStreams: true, // VideoBox:show
  joystickEnabled: false, // Layout:joystickState | Joystick:enable | Teleop:enableJoystick
  joystickConfig: { // Joystick:absoluteMaxX | Joystick:absoluteMaxY
    maxX: 0,
    maxY: 0,
  },
  rates: {
    joystickCanvasRefreshRate: 60, // Joystick:canvasRefreshRate
    joystickPositionRefreshRate: 100, // Joystick:operatorCommandInterval
    patrolCanvasRefreshRate: 60, // PatrolMap:CanvasRefreshRate
  },
  htmlElement: {
    cameraId: 'camera-videobox-html-id',
    camera: null,
    mapId: 'map-videobox-html-id',
    map: null,
    patrolId: 'patrol-videobox-html-id',
    patrol: null,
  },
  patrol: {
    enable: false, // PatrolMap:enable
    waypointList: [], // Patrol:waypointList | PatrolMap:waypointList
    // SaveLoad:patrolList | Patrol:patrolList
    patrolList: JSON.parse('[{"Name":"Test","waypoints":[{"x":593.2924107142857,"y":323.21428571428567,"yaw":0},{"x":550.4352678571429,"y":303.57142857142856,"yaw":0},{"x":518.2924107142858,"y":435.71428571428567,"yaw":0}]}]'),
  },
};
