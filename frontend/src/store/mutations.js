/**
 * The root store mutations.
 *
 * @module Store
 * @exports
 */

/**
 * @typedef {Object} Coordinate
 * @property {Number} x - The x coordinate in pixel
 * @property {Number} y - The y coordinate in pixel
 * @property {Number} yaw - The waypoint angle
 */

/**
 * @typedef {Object} Waypoint
 * @property {Coordinate} coordinate - The coordinate object
 * @property {String} label - A label for the waypoints
 * @property {Number} hold_time_s - A timeout for the waypoint
 */

/**
 * @typedef {Object} Patrol
 * @property {String} name - The name of the patrol
 * @property {String} robot - The robot id
 * @property {String} description_text - The description of the patrol
 * @property {String} last_modified - Date time in iso string
 * @property {Array<Waypoint>} waypoints - A list of waypoint
 */

/**
 * @typedef {Object} Schedule
 * @property {String} name - The name of the patrol
 * @property {String} robot - The robot id
 * @property {String} description_text - The description of the patrol
 * @property {String} patrol - The patrol id
 * @property {String} last_modified - Date time in iso string
 * @property {String} cron - The cron representation of the schedule
 * @property {Number} timeout_s - The amount of time before the schedule is stopped
 * @property {Number} repetitions - The amount to repeat the patrol
 * @property {Boolean} enabled - Enable schedule
 */

/**
 * @typedef {Object} Schedule
 */

export default {
  /**
   * Set the info of the robot the user is connected to.
   *
   * @param {Vuex} state - Vuex states
   * @param {Object} robot - The robot object (@see easyrtc-client#robot)
   */
  setConnectedRobot(state, robot) {
    state.currentRobot.id.db = '';
    state.currentRobot.name = robot.robotName;
    state.currentRobot.id.client = robot.robotId;
    if (state.currentRobot.name) {
      for (const r of state.database.robots) {
        if (state.currentRobot.name === r.name) {
          state.currentRobot.id.db = r.id;
        }
      }
    } else {
      state.currentRobot.id.db = '';
    }
  },
  /**
   * Sets joystick enable value to true.
   * @param {Vuex} state - Vuex states
   */
  enableJoystick(state) {
    state.joystickEnabled = true;
  },
  /**
   * Sets joystick enable value to false.
   * @param {Vuex} state - Vuex states
   */
  disableJoystick(state) {
    state.joystickEnabled = false;
  },
  /**
   * Set the interval for docking execution.
   * @param {Vuex} state - Vuex states
   * @param {NodeJS.Timeout} interval - The interval object
   */
  setDockingProcess(state, interval) {
    state.dockingInterval = interval;
  },
  /**
   * Clears the docking procedure interval.
   * @param {Vuex} state - Vuex states
   */
  clearDockingProcess(state) {
    if (state.dockingInterval) {
      clearInterval(state.dockingInterval);
    }
    state.dockingInterval = '';
  },
  /**
   * Sets the html video element for the camera.
   * @param {Vuex} state - Vuex states
   * @param {HTMLVideoElement} element - The html element
   */
  setCameraHTMLElement(state, element) {
    state.htmlElement.camera = element;
  },
  /**
   * Clears the html video element of the camera.
   * @param {Vuex} state - Vuex states
   */
  clearCameraHTMLElement(state) {
    state.htmlElement.camera = null;
  },
  /**
   * Sets the html video element for the map (teleop).
   * @param {Vuex} state - Vuex states
   * @param {HTMLVideoElement} element - The html element
   */
  setMapHTMLElement(state, element) {
    state.htmlElement.map = element;
  },
  /**
   * Clears the html video element of the map (teleop).
   * @param {Vuex} state - Vuex states
   */
  clearMapHTMLElement(state) {
    state.htmlElement.map = null;
  },
  /**
   * Sets the html video element for the map (patrol).
   * @param {Vuex} state - Vuex states
   * @param {HTMLVideoElement} element - The html element
   */
  setPatrolHTMLElement(state, element) {
    state.htmlElement.patrol = element;
  },
  /**
   * Clears the html video element of the map (patrol).
   * @param {Vuex} state - Vuex states
   */
  clearPatrolHTMLElement(state) {
    state.htmlElement.patrol = null;
  },
  /**
   * Sets the html video element for the map (events).
   * @param {Vuex} state - Vuex states
   * @param {HTMLVideoElement} element - The html element
   */
  setEventHTMLElement(state, element) {
    state.htmlElement.event = element;
  },
  /**
   * Clears the html video element of the map (events).
   * @param {Vuex} state - Vuex states
   */
  clearEventHTMLElement(state) {
    state.htmlElement.event = null;
  },
  /**
   * Adds a waypoint to the list.
   * @param {Vuex} state - Vuex states
   * @param {Object} add - Object with wp:Waypoint and index:Number(optionnal)
   */
  addWaypoint(state, add) {
    if ('index' in add) {
      state.patrol.current.obj.waypoints.splice(add.index, 0, add.wp);
    } else {
      state.patrol.current.obj.waypoints.push(add.wp);
    }
  },
  /**
   * Sets the label of a waypoint.
   * @param {Vuex} state - Vuex states
   * @param {Onject} data - An object with value:String and index:Number
   */
  setWaypointLabel(state, data) {
    state.patrol.current.obj.waypoints[data.index].label = data.value;
  },
  /**
   * Sets the timeout/hold value for a waypoint.
   * @param {Vuex} state - Vuex states
   * @param {Index&Waypoint} data - An object with value:Number and index:Number
   */
  setWaypointHold(state, data) {
    state.patrol.current.obj.waypoints[data.index].hold_time_s = data.value;
  },
  /**
   * Updates a waypoint values.
   * @param {Vuex} state - Vuex states
   * @param {Index&Waypoint} update - An object with wp:Waypoint and index:Number
   */
  updateWaypoint(state, update) {
    state.patrol.current.obj.waypoints.splice(update.index, 1, update.wp);
  },
  /**
   * Removes a waypoint from the list.
   * @param {Vuex} state - Vuex states
   * @param {Number} index - The index of the waypoint to remove
   */
  removeWaypoint(state, index) {
    state.patrol.current.obj.waypoints.splice(index, 1);
  },
  /**
   * Reorders a waypoint. It first remove it from the list than reinsert at the give index.
   * @param {Vuex} state - Vuex states
   * @param {Object} data - An object with oldIndex:Number and newIndex:Number
   */
  reorderWaypoint(state, data) {
    if (data.newIndex >= 0 && data.newIndex < state.patrol.current.obj.waypoints.length) {
      const wp = state.patrol.current.obj.waypoints.splice(data.oldIndex, 1);
      state.patrol.current.obj.waypoints.splice(data.newIndex, 0, wp[0]);
    }
  },
  /**
   * Clears the waypoint list.
   * @param {Vuex} state - Vuex states
   */
  clearWaypointList(state) {
    state.patrol.current.obj.waypoints = [];
  },
  /**
   * Fills the waypoint list.
   * @param {Vuex} state - Vuex states
   * @param {Array<Waypoint>} waypointList - List of waypoints.
   */
  fillWaypointList(state, waypointList) {
    state.patrol.current.obj.waypoints = waypointList;
  },
  /**
   * Adds patrol to list.
   * @param {Vuex} state - Vuex states
   * @param {Patrol} patrol - The patrol object
   */
  addPatrol(state, patrol) {
    state.patrol.list.push(patrol);
  },
  /**
   * Updates patrol in list.
   * @param {Vuex} state - Vuex states
   * @param {Patrol} update - The new patrol object
   */
  updatePatrol(state, update) {
    state.patrol.list.splice(update.index, 1, update.patrol);
  },
  /**
   * Removes patrol in list.
   * @param {Vuex} state - Vuex states
   * @param {Number} index - The index of the patrol to remove from the list
   */
  removePatrol(state, index) {
    state.patrol.list.splice(index, 1);
  },
  /**
   * Clears the patrol list.
   * @param {Vuex} state - Vuex states
   */
  clearPatrols(state) {
    state.patrol.list = [];
  },
  /**
   * Sets the current patrol id.
   * @param {Vuex} state - Vuex states
   * @param {String} id - The patrol id.
   */
  setCurrentPatrolId(state, id) {
    state.patrol.current.id = id;
  },
  /**
   * Sets the current patrol values.
   * @param {Vuex} state - Vuex states
   * @param {Patrol} obj - The patrol object
   */
  setCurrentPatrol(state, obj) {
    const keys = Object.keys(obj);
    const pKeys = Object.keys(state.patrol.current.obj);
    for (const key of keys) {
      if (pKeys.includes(key)) {
        state.patrol.current.obj[key] = obj[key];
      }
    }
  },
  /**
   * Clears the current patrols values.
   * @param {Vuex} state - Vuex states
   */
  clearCurrentPatrol(state) {
    state.patrol.current.obj = {
      name: '',
      robot: '',
      description_text: '',
      last_modified: '',
      waypoints: [],
    };
  },
  /**
   * Adds schedule to list.
   * @param {Vuex} state - Vuex states
   * @param {Schedule} schedule - The schedule object
   */
  addSchedule(state, schedule) {
    state.schedule.list.push(schedule);
  },
  /**
   * Updates shcedules in list.
   * @param {Vuex} state - Vuex states
   * @param {Schedule} update - The new schedule object
   */
  updateSchedule(state, update) {
    state.schedule.list.splice(update.index, 1, update.schedule);
  },
  /**
   * Removes schedule from list.
   * @param {Vuex} state - Vuex states
   * @param {Number} index - The index of the schedule to remove.
   */
  removeSchedule(state, index) {
    state.schedule.list.splice(index, 1);
  },
  /**
   * Clears the schdule list.
   * @param {Vuex} state - Vuex states
   */
  clearSchedules(state) {
    state.schedule.list = [];
  },
  /**
   * Sets the current schedule id.
   * @param {Vuex} state - Vuex states
   * @param {String} id - The id of the schedule.
   */
  setCurrentScheduleId(state, id) {
    state.schedule.current.id = id;
  },
  /**
   * Sets the current schedule values.
   * @param {Vuex} state - Vuex states
   * @param {Schedule} obj - The schedule object.
   */
  setCurrentSchedule(state, obj) {
    const keys = Object.keys(obj);
    const sKeys = Object.keys(state.schedule.current.obj);
    for (const key of keys) {
      if (sKeys.includes(key)) {
        state.schedule.current.obj[key] = obj[key];
      }
    }
  },
  /**
   * Clears the current schedule values.
   * @param {Vuex} state - Vuex states
   */
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
      enabled: true,
    };
  },
  /**
   * Increases the map zoom by 0.1 to a max of 2.
   * @param {Vuex} state - Vuex states
   */
  increaseMapZoom(state) {
    if (state.mapZoom < 2) {
      state.mapZoom = Number((state.mapZoom + 0.1).toFixed(1));
    } else {
      state.mapZoom = 2;
    }
  },
  /**
   * Decreases the map zoom by 0.1 to min of 1.
   * @param {Vuex} state - Vuex states
   */
  decreaseMapZoom(state) {
    if (state.mapZoom > 1) {
      state.mapZoom = Number((state.mapZoom - 0.1).toFixed(1));
    } else {
      state.mapZoom = 1;
    }
  },
  /**
   * Sets the map size values.
   * @param {Vuex} state - Vuex states
   * @param {Object} size - The map size.
   */
  setMapSize(state, size) {
    state.mapSize = size;
  },
};
