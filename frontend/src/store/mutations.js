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
  setDockingProcess(state, interval) {
    state.dockingInterval = interval;
  },
  clearDockingProcess(state) {
    if (state.dockingInterval) {
      clearInterval(state.dockingInterval);
    }
    state.dockingInterval = '';
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
      state.waypoints.list.splice(add.index, 0, add.wp);
    } else {
      state.waypoints.list.push(add.wp);
    }
  },
  updateWaypoint(state, update) {
    state.waypoints.list.splice(update.index, 1, update.wp);
  },
  removeWaypoint(state, index) {
    state.waypoints.list.splice(index, 1);
  },
  clearWaypointList(state) {
    state.waypoints.list = [];
  },
  fillWaypointList(state, waypointList) {
    state.waypoints.list = waypointList;
  },
  addPatrol(state, patrol) {
    state.patrol.list.push(patrol);
  },
  updatePatrol(state, update) {
    state.patrol.list.splice(update.index, 1, update.patrol);
  },
  removePatrol(state, index) {
    state.patrol.list.splice(index, 1);
  },
  clearPatrols(state) {
    state.patrol.list = [];
  },
  setCurrentPatrolId(state, id) {
    state.patrol.current.id = id;
  },
  setCurrentPatrol(state, obj) {
    const keys = Object.keys(obj);
    const pKeys = Object.keys(state.patrol.current.obj);
    for (const key of keys) {
      if (pKeys.includes(key)) {
        state.patrol.current.obj[key] = obj[key];
      }
    }
  },
  clearCurrentPatrol(state) {
    state.patrol.current.obj = {
      name: '',
      robot: '',
      description_text: '',
      last_modified: '',
      waypoints: [],
    };
  },
  addSchedule(state, schedule) {
    state.schedule.list.push(schedule);
  },
  updateSchedule(state, update) {
    state.schedule.list.splice(update.index, 1, update.schedule);
  },
  removeSchedule(state, index) {
    state.schedule.list.splice(index, 1);
  },
  clearSchedules(state) {
    state.schedule.list = [];
  },
  setCurrentScheduleId(state, id) {
    state.schedule.current.id = id;
  },
  setCurrentSchedule(state, obj) {
    const keys = Object.keys(obj);
    const sKeys = Object.keys(state.schedule.current.obj);
    for (const key of keys) {
      if (sKeys.includes(key)) {
        state.schedule.current.obj[key] = obj[key];
      }
    }
  },
  clearCurrentSchedule(state) {
    state.schedule.current.obj = {
      name: '',
      robot: '',
      description_text: '',
      patrol: '',
      last_modified: '',
      cron: '',
      timeout_s: '',
      repetitions: '',
      enabled: false,
    };
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
