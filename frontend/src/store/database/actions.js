/* eslint-disable no-underscore-dangle */
import request from 'request-promise';

/**
 * The database module actions.
 * Depends on the request-promise module.
 *
 * @module Database
 * @exports
 */
export default {
  /**
   * Sets the event image url.
   * @param {Vuex} Vuex
   * @param {String} file - the filename of the image
   */
  setEventImageURL({ getters, commit }, file) {
    const url = `${getters.uri}/files/${file}`;
    commit('setEventImageURL', url);
  },
  /**
   * Initiates local data by querying the database with defaut param.
   * @param {Vuex} Vuex
   * @returns {Promise}
   */
  initLocalData({ state, commit, dispatch }) {
    /**
     * Steps
     *
     * 1 - Get All Robots and their ids
     * 2 - Get All Patrol Name (aka all patrol for each robots)
     * 3 - Get All Schedules (same as Patrols)
     * 4 - Get All Events
     */
    return new Promise((resolve, reject) => {
      commit('queryStarted');
      dispatch('queryRobots')
        .then(() => {
          commit('setRobotFilter', ['all']);
          commit('setEventFilter', state.defaultFilter);
          dispatch('queryEvents')
            .then(() => {
              commit('queryFinished');
              dispatch('queryPatrols')
                .then(() => {
                  dispatch('querySchedules')
                    .then(() => {
                      resolve();
                    }).catch((err) => {
                      reject(err);
                    });
                }).catch((err) => {
                  reject(err);
                });
            }).catch((err) => {
              reject(err);
              commit('errorDuringQuery');
              commit('queryFinished');
            });
        }).catch((err) => {
          reject(err);
          commit('errorDuringQuery');
          commit('queryFinished');
        });
    });
  },
  /**
   * Filters the event with new query.
   * @param {Vuex} Vuex
   */
  filterEvents({ commit, dispatch }) {
    commit('resetEvents');
    commit('queryStarted');
    dispatch('queryEvents')
      .then(() => {
        commit('queryFinished');
      }).catch((err) => {
        console.log(err);
        commit('errorDuringQuery');
        commit('queryFinished');
      });
  },
  /**
   * Queries the robots from the database.
   *
   * @param {Vuex} Vuex
   * @returns {Promise}
   */
  queryRobots({ getters, commit }) {
    const req = {
      uri: `${getters.uri}/robots`,
      headers: {
        'User-Agent': 'Request-Promise',
      },
      json: true,
    };

    return new Promise((resolve, reject) => {
      request(req)
        .then((result) => {
          result.forEach((robot) => {
            commit('addRobot', { name: robot.name, id: robot._id });
          });
          resolve();
        }).catch((err) => {
          reject(err);
        });
    });
  },
  /**
   * Queries the patrols from the database.
   *
   * @param {Vuex} Vuex
   * @returns {Promise}
   */
  // eslint-disable-next-line object-curly-newline
  queryPatrols({ state, commit, dispatch }) {
    const { robots } = state;
    const options = {
      robots,
      index: 0,
    };
    commit('clearPatrols', '', { root: true });
    return new Promise((resolve, reject) => {
      dispatch('_queryPatrol', options)
        .then(() => {
          resolve();
        }).catch((err) => {
          reject(err);
        });
    });
  },
  /**
   * Self-calling promise that iterates over the robot list to get all patrols on database.
   * @param {Vuex} Vuex
   * @param {Object} options - The option/filter object
   * @returns {Promise}
   */
  _queryPatrol({ getters, dispatch, commit }, options) {
    return new Promise((resolve, reject) => {
      const _options = {};
      Object.assign(_options, options);
      if (_options.index >= _options.robots.length) {
        resolve();
      }
      const currentRobot = options.robots[options.index];
      _options.index += 1;
      const req = {
        uri: `${getters.uri}/robots/${currentRobot.id}/patrols`,
        headers: {
          'User-Agent': 'Request-Promise',
        },
        json: true,
      };
      // Request
      request(req)
        .then((result) => {
          result.forEach((patrol) => {
            const p = {
              name: patrol.name,
              info: {
                robotId: currentRobot.id,
                patrolId: patrol._id,
              },
            };
            commit('addPatrol', p, { root: true });
          });
          return dispatch('_queryPatrol', _options)
            .then(() => {
              resolve();
            }).catch((err) => {
              reject(err);
            });
        }).catch((err) => {
          reject(err);
        });
    });
  },
  /**
   * Gets all the schedule from the database.
   * @param {Vuex} Vuex
   * @returns {Promise}
   */
  querySchedules({ state, commit, dispatch }) {
    commit('clearSchedules', '', { root: true });
    const { robots } = state;
    const options = {
      robots,
      index: 0,
    };
    return new Promise((resolve, reject) => {
      dispatch('_querySchedule', options)
        .then(() => {
          resolve();
        }).catch((err) => {
          reject(err);
        });
    });
  },
  /**
   * Self-calling promise that iterates over the robot list to get all schedules on database.
   * @param {Vuex} Vuex
   * @param {Object} options - The options/filter object
   * @returns {Promise}
   */
  _querySchedule({ getters, dispatch, commit }, options) {
    return new Promise((resolve, reject) => {
      const _options = {};
      Object.assign(_options, options);
      if (_options.index >= _options.robots.length) {
        resolve();
      }
      const currentRobot = options.robots[options.index];
      _options.index += 1;
      const req = {
        uri: `${getters.uri}/robots/${currentRobot.id}/schedules`,
        headers: {
          'User-Agent': 'Request-Promise',
        },
        json: true,
      };
      // Request
      request(req)
        .then((result) => {
          result.forEach((schedule) => {
            const s = {
              name: schedule.name,
              info: {
                robotId: currentRobot.id,
                patrolId: schedule.patrol,
                scheduleId: schedule._id,
              },
            };
            commit('addSchedule', s, { root: true });
          });
          return dispatch('_querySchedule', _options)
            .then(() => {
              resolve();
            }).catch((err) => {
              reject(err);
            });
        }).catch((err) => {
          reject(err);
        });
    });
  },
  /**
   * Get the events from the database.
   * @param {Vuex} Vuex
   * @returns {Promise}
   */
  queryEvents({ state, commit, dispatch }) {
    commit('resetEvents');
    const robots = state.robotFilter;
    const options = {
      robots,
      index: 0,
      filters: state.eventFilter,
    };
    return new Promise((resolve, reject) => {
      dispatch('_queryEvents', options)
        .then(() => {
          resolve();
        }).catch((err) => {
          reject(err);
        });
    });
  },
  /**
   * Self-calling promise that iterates over the robot list to get all events on database.
   * @param {Vuex} Vuex
   * @param {Object} options - The options/filter object
   * @returns {Promise}
   */
  _queryEvents({ getters, commit, dispatch }, options) {
    return new Promise((resolve, reject) => {
      const _options = {};
      Object.assign(_options, options);
      if (_options.index >= _options.robots.length) {
        resolve();
      }
      const currentRobot = options.robots[options.index];
      _options.index += 1;
      const req = {
        uri: `${getters.uri}/robots/${currentRobot.id}/events`,
        headers: {
          'User-Agent': 'Request-Promise',
        },
        json: true,
      };
      Object.assign(req.headers, options.filters);
      // Request
      request(req)
        .then((result) => {
          result.forEach((event) => {
            commit('addEvent', event);
          });
          return dispatch('_queryEvents', _options)
            .then(() => {
              resolve();
            }).catch((err) => {
              reject(err);
            });
        }).catch((err) => {
          reject(err);
        });
    });
  },
  /**
   * Saves the patrol on database using post or updates it using put.
   * @param {Vuex} Vuex
   * @param {Object} patrol - The patrol object to save
   * @returns {Promise}
   */
  savePatrol({ getters, commit }, patrol) {
    const req = {
      uri: `${getters.uri}/robots/${patrol.obj.robot}/patrols${patrol.id ? `/${patrol.id}` : ''}`,
      method: (patrol.id ? 'PUT' : 'POST'),
      headers: {
        'User-Agent': 'Request-Promise',
      },
      json: true,
      body: patrol.obj,
    };
    return new Promise((resolve, reject) => {
      request(req)
        .then((result) => {
          commit('setCurrentPatrolId', result._id, { root: true });
          resolve();
        }).catch((err) => {
          reject(err);
        });
    });
  },
  /**
   * Saves the schedule on database using post or updates it using put.
   * @param {Vuex} Vuex
   * @param {Object} schedule - The schedule object to save
   * @returns {Promise}
   */
  saveSchedule({ getters, commit }, schedule) {
    const req = {
      uri: `${getters.uri}/robots/${schedule.obj.robot}/schedules${schedule.id ? `/${schedule.id}` : ''}`,
      method: (schedule.id ? 'PUT' : 'POST'),
      headers: {
        'User-Agent': 'Request-Promise',
      },
      json: true,
      body: schedule.obj,
    };
    return new Promise((resolve, reject) => {
      request(req)
        .then((result) => {
          commit('setCurrentScheduleId', result._id, { root: true });
          resolve(result);
        }).catch((err) => {
          reject(err);
        });
    });
  },
  /**
   * Queries the patrol data (waypoint list, description, etc.).
   * @param {Vuex} Vuex
   * @param {Object} info - The info object of the patrol to get
   */
  getPatrol({ getters, commit }, info) {
    commit('clearCurrentPatrol', '', { root: true });
    const req = {
      uri: `${getters.uri}/robots/${info.robotId}/patrols/${info.patrolId}`,
      headers: {
        'User-Agent': 'Request-Promise',
      },
      json: true,
    };
    request(req)
      .then((result) => {
        const wp = [];
        for (const waypoint of result.waypoints) {
          wp.push(waypoint.coordinate);
        }
        commit('setCurrentPatrolId', result._id, { root: true });
        commit('setCurrentPatrol', result, { root: true });
      }).catch((err) => {
        console.log(err);
      });
  },
  /**
   * Queries the schedule data (cron, repetitions, etc.).
   * @param {Vuex} Vuex
   * @param {Object} info - The info object of the schedule to get
   */
  getSchedule({ getters, commit }, info) {
    commit('clearCurrentSchedule', '', { root: true });
    const req = {
      uri: `${getters.uri}/robots/${info.robotId}/schedules/${info.scheduleId}`,
      headers: {
        'User-Agent': 'Request-Promise',
      },
      json: true,
    };
    request(req)
      .then((result) => {
        commit('setCurrentScheduleId', result._id, { root: true });
        commit('setCurrentSchedule', result, { root: true });
      }).catch((err) => {
        console.log(err);
      });
  },
  /**
   * Removes the patrol from the database using DELETE
   * @param {Vuex} Vuex
   * @param {Object} patrol - The patrol to remove
   * @returns {Promise}
   */
  removePatrol({ getters, commit }, patrol) {
    const req = {
      uri: `${getters.uri}/robots/${patrol.obj.robot}/patrols/${patrol.id}`,
      method: 'DELETE',
      headers: {
        'User-Agent': 'Request-Promise',
      },
      json: true,
    };
    return new Promise((resolve, reject) => {
      request(req)
        .then(() => {
          commit('setCurrentPatrolId', '', { root: true });
          commit('clearCurrentPatrol', '', { root: true });
          resolve();
        }).catch((err) => {
          reject(err);
        });
    });
  },
  /**
   * Removes the schedule from the database using DELETE.
   * @param {Vuex} Vuex
   * @param {Object} schedule - The schedule to remove
   * @returns {Promise}
   */
  removeSchedule({ getters, commit }, schedule) {
    const req = {
      uri: `${getters.uri}/robots/${schedule.obj.robot}/schedules/${schedule.id}`,
      method: 'DELETE',
      headers: {
        'User-Agent': 'Request-Promise',
      },
      json: true,
    };
    return new Promise((resolve, reject) => {
      request(req)
        .then(() => {
          commit('setCurrentScheduleId', '', { root: true });
          commit('clearCurrentSchedule', '', { root: true });
          resolve();
        }).catch((err) => {
          reject(err);
        });
    });
  },
};
