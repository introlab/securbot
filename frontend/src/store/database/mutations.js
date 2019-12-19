/**
 * The database module mutations.
 *
 * @module Database
 * @exports
 */
export default {
  /**
   * Set querying to true.
   * @param {Vuex} state - Vuex states
   */
  queryStarted(state) {
    state.queryingDB = true;
  },
  /**
   * Set querying to false.
   * @param {Vuex} state - Vuex states
   */
  queryFinished(state) {
    state.queryingDB = false;
  },
  /**
   * Reset query error.
   * @param {Vuex} state - Vuex states
   */
  resetQuery(state) {
    state.errorDuringQuery = false;
  },
  /**
   * Set query error.
   * @param {Vuex} state - Vuex states
   */
  errorDuringQuery(state) {
    state.errorDuringQuery = true;
  },
  /**
   * Add robot to robot list.
   * @param {Vuex} state - Vuex states
   * @param {*} robot - The robot to add
   */
  addRobot(state, robot) {
    state.robots.push(robot);
  },
  /**
   * Add event to event list.
   * @param {Vuex} state - Vuex states
   * @param {*} event - The event to add
   */
  addEvent(state, event) {
    state.events.push(event);
  },
  /**
   * Clears event list.
   * @param {Vuex} state - Vuex states
   */
  resetEvents(state) {
    state.events = [];
  },
  /**
   * Sets the image url.
   * @param {Vuex} state - Vuex states
   * @param {*} url - The url
   */
  setEventImageURL(state, url) {
    state.eventImageURL = url;
  },
  /**
   * Clears the image url.
   * @param {Vuex} state - Vuex states
   */
  clearEventImageURL(state) {
    state.eventImageURL = '';
  },
  /**
   * Resets the robot filter.
   * @param {Vuex} state - Vuex states
   */
  resetRobotFilter(state) {
    state.robotFilter = [];
  },
  /**
   * Set the robot filter by adding robot to the empty filter list.
   * @param {Vuex} state - Vuex states
   * @param {*} robots - The robots to add to the filter list
   */
  setRobotFilter(state, robots) {
    if (robots && robots.length) {
      if (robots.includes('all')) {
        state.robotFilter = state.robots;
      } else {
        state.robotFilter = [];
        for (const r of state.robots) {
          if (robots.includes(r.id)) {
            state.robotFilter.push(r);
          }
        }
      }
    }
  },
  /**
   * Reset the event filter.
   * @param {Vuex} state - Vuex states
   */
  resetEventFilter(state) {
    state.eventFilter = {};
  },
  /**
   * Set the event filter. Can be used to reactively set parameters.
   * @param {Vuex} state - Vuex states
   * @param {*} filters - The filter object that includes the properties and their values
   */
  setEventFilter(state, filters) {
    const f = {};
    const prevFilter = JSON.parse(JSON.stringify(state.eventFilter));
    const keys = Object.keys(filters);
    if (keys.length) {
      for (const key of keys) {
        switch (key) {
          case 'tag_not':
            f.tag_not = filters.tag_not;
            break;
          case 'tag_and':
            f.tag_and = filters.tag_and;
            break;
          case 'search_expression':
            f.search_expression = filters.search_expression;
            break;
          case 'alert':
            f.alert = filters.alert;
            break;
          case 'viewed':
            f.viewed = filters.viewed;
            break;
          case 'before':
            if (typeof filters.before === 'number') {
              // eslint-disable-next-line max-len
              f.before = new Date(new Date().getTime() - filters.before).toISOString();
            } else if (typeof filters.before === 'string') {
              f.before = new Date(filters.before).toISOString();
            } else {
              console.log('Before date format is unsable... Ignoring...');
            }
            break;
          case 'after':
            if (typeof filters.after === 'number') {
              // eslint-disable-next-line max-len
              f.after = new Date(new Date().getTime() - filters.after).toISOString();
            } else if (typeof filters.after === 'string') {
              f.after = new Date(filters.after).toISOString();
            } else {
              console.log('After date format is unsable... Ignoring...');
            }
            break;
          default:
            break;
        }
      }
      Object.assign(prevFilter, f);
      state.eventFilter = prevFilter;
    } else {
      state.eventFilter = {};
    }
  },
};
