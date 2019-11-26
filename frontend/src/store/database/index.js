/* eslint-disable no-underscore-dangle */
import request from 'request-promise';

export default {
  namespaced: true,
  state: {
    apiPath: process.env.VUE_APP_DB_PATH,
    serverURL: (
      process.env.VUE_APP_DEV_DB_URL
        ? process.env.VUE_APP_DEV_DB_URL
        : process.env.VUE_APP_SERVER_URL
    ),
    queryingDB: false,
    errorDuringQuery: false,
    events: [],
    eventImageURL: '',
    robots: [],
    robotFilter: [],
    tagList: ['blue', 'yellow', 'red', 'green'],
    defaultFilter: {
      after: 7 * 24 * 60 * 60 * 1000,
      viewed: false,
      alert: false,
    },
    predefFiltersFormat: [
      {
        name: 'Last 24 Hours',
        filters: {
          after: 24 * 60 * 60 * 1000,
        },
      },
      {
        name: 'Last 7 Days',
        filters: {
          after: 7 * 24 * 60 * 60 * 1000,
        },
      },
      {
        name: 'Last 30 Days',
        filters: {
          after: 30 * 24 * 60 * 60 * 1000,
        },
      },
      {
        name: 'Current Month',
        filters: {
          after: new Date(new Date().setDate(1)).toISOString(),
        },
      },
      {
        name: 'All',
        filters: {},
      },
      {
        name: 'Alerts from Last 30 Days',
        filters: {
          alert: true,
          after: 30 * 24 * 60 * 60 * 1000,
        },
      },
      {
        name: 'New Events from Last 30 Days',
        filters: {
          viewed: false,
          after: 30 * 24 * 60 * 60 * 1000,
        },
      },
    ],
    eventFilter: {},
  },
  getters: {
    uri: state => `${(state.serverURL.startsWith('http') ? '' : 'http://')}${state.serverURL}${state.apiPath}`,
    eventsWaypoints: (state) => {
      const wpl = [];
      for (const event of state.events) {
        wpl.push(event.location_coordinate);
      }
      return wpl;
    },
  },
  mutations: {
    queryStarted(state) {
      state.queryingDB = true;
    },
    queryFinished(state) {
      state.queryingDB = false;
    },
    resetQuery(state) {
      state.errorDuringQuery = false;
    },
    errorDuringQuery(state) {
      state.errorDuringQuery = true;
    },
    addRobot(state, robot) {
      state.robots.push(robot);
    },
    addEvent(state, event) {
      state.events.push(event);
    },
    resetEvents(state) {
      state.events = [];
    },
    setEventImageURL(state, url) {
      state.eventImageURL = url;
    },
    clearEventImageURL(state) {
      state.eventImageURL = '';
    },
    resetRobotFilter(state) {
      state.robotFilter = [];
    },
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
    resetEventFilter(state) {
      state.eventFilter = {};
    },
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
  },
  actions: {
    setEventImageURL({ getters, commit }, file) {
      const url = `${getters.uri}/files/${file}`;
      commit('setEventImageURL', url);
    },
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
     *
     *
     * @param {*} { getters, commit }
     * @returns
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
     *
     *
     * @param {*} { state, commit }
     * @returns
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
    // eslint-disable-next-line object-curly-newline
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
  },
};
