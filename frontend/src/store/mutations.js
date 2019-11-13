export default {
  setConnectedRobot(state, robot) {
    console.log('Setting Up robot');
    state.currentRobot.id.db = '';
    state.currentRobot.name = robot.robotName;
    state.currentRobot.id.client = robot.robotId;
    for (const r of state.database.robots) {
      if (state.currentRobot.name === r.name) {
        state.currentRobot.id.db = r.id;
      }
    }
  },
  showStreams(state) {
    state.showStreams = true;
  },
  hideStreams(state) {
    state.showStreams = false;
  },
  enableJoystick(state) {
    state.joystickEnabled = true;
  },
  disableJoystick(state) {
    state.joystickEnabled = false;
  },
  setCameraHTMLElement(state, element) {
    state.htmlElement.camera = element;
  },
  clearCameraHTMLElement(state) {
    state.htmlElement.camera = null;
  },
  setMapHTMLElement(state, element) {
    state.htmlElement.map = element;
  },
  clearMapHTMLElement(state) {
    state.htmlElement.map = null;
  },
  setPatrolHTMLElement(state, element) {
    state.htmlElement.patrol = element;
  },
  clearPatrolHTMLElement(state) {
    state.htmlElement.patrol = null;
  },
  setEventHTMLElement(state, element) {
    state.htmlElement.event = element;
  },
  clearEventHTMLElement(state) {
    state.htmlElement.event = null;
  },
  addWaypoint(state, add) {
    if ('index' in add) {
      state.patrol.waypointList.splice(add.index, 0, add.wp);
    } else {
      state.patrol.waypointList.push(add.wp);
    }
  },
  updateWaypoint(state, update) {
    state.patrol.waypointList.splice(update.index, 1, update.wp);
  },
  removeWaypoint(state, index) {
    state.patrol.waypointList.splice(index, 1);
  },
  clearWaypointList(state) {
    state.patrol.waypointList = [];
  },
  fillWaypointList(state, waypointList) {
    state.patrol.waypointList = waypointList;
  },
  addPatrol(state, patrol) {
    state.patrol.patrolList.push(patrol);
  },
  updatePatrol(state, update) {
    state.patrol.patrolList.splice(update.index, 1, update.patrol);
  },
  removePatrol(state, index) {
    state.patrol.patrolList.splice(index, 1);
  },
  clearPatrols(state) {
    state.patrol.patrolList = [];
  },
  addSchedule(state, schedule) {
    state.patrol.scheduleList.push(schedule);
  },
  updateSchedule(state, update) {
    state.patrol.scheduleList.splice(update.index, 1, update.schedule);
  },
  removeSchedule(state, index) {
    state.patrol.scheduleList.splice(index, 1);
  },
  clearSchedules(state) {
    state.patrol.scheduleList = [];
  },
  increaseMapZoom(state) {
    if (state.mapZoom < 2) {
      state.mapZoom = Number((state.mapZoom + 0.1).toFixed(1));
    } else {
      state.mapZoom = 2;
    }
  },
  decreaseMapZoom(state) {
    if (state.mapZoom > 1) {
      state.mapZoom = Number((state.mapZoom - 0.1).toFixed(1));
    } else {
      state.mapZoom = 1;
    }
  },
  setMapSize(state, size) {
    state.mapSize = size;
  },
};
