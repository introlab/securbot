/* eslint-disable no-underscore-dangle */
import request from 'request-promise';

export default {
  namespaced: true,
  state: {
    apiPath: '/db',
    queryingDB: false,
    errorDuringQuery: false,
    events: [],
    headers: [
      { key: 'name', label: 'Robot' },
      { key: 'object', label: 'Object' },
      { key: 'context', label: 'Context' },
      { key: 'description_text', label: 'Description' },
      { key: 'tags', label: 'Tags' },
      { key: 'time', label: 'DateTime' },
      { key: 'image', label: 'Image' },
    ],
    robots: [],
    robotFilter: [],
    tagList: ['red', 'yellow', 'blue', 'green', 'a', 'b', 'c'],
    defaultFilter: {
      before: new Date().toISOString(),
      after: new Date(new Date().getTime() - (7 * 24 * 60 * 60 * 1000)).toISOString(),
      viewed: false,
      alert: true,
    },
    predefFilters: [
      {
        name: 'Last 24 Hours',
        filters: {
          tag_and: [],
          tag_not: [],
          search_expression: '',
          alert: '',
          viewed: '',
          before: new Date().toISOString(),
          after: new Date(new Date().getTime() - (24 * 60 * 60 * 1000)).toISOString(),
        },
      },
      {
        name: 'Last 7 Days',
        filters: {
          tag_and: [],
          tag_not: [],
          search_expression: '',
          alert: '',
          viewed: '',
          before: new Date().toISOString(),
          after: new Date(new Date().getTime() - (7 * 24 * 60 * 60 * 1000)).toISOString(),
        },
      },
      {
        name: 'Last 30 Days',
        filters: {
          tag_and: [],
          tag_not: [],
          search_expression: '',
          alert: '',
          viewed: '',
          before: new Date().toISOString(),
          after: new Date(new Date().getTime() - (30 * 24 * 60 * 60 * 1000)).toISOString(),
        },
      },
      {
        name: 'Alerts from Last 30 Days',
        filters: {
          tag_and: [],
          tag_not: [],
          search_expression: '',
          alert: true,
          viewed: '',
          before: new Date().toISOString(),
          after: new Date(new Date().getTime() - (30 * 24 * 60 * 60 * 1000)).toISOString(),
        },
      },
      {
        name: 'New Events from Last 30 Days',
        filters: {
          tag_and: [],
          tag_not: [],
          search_expression: '',
          alert: '',
          viewed: false,
          before: new Date().toISOString(),
          after: new Date(new Date().getTime() - (30 * 24 * 60 * 60 * 1000)).toISOString(),
        },
      },
      {
        name: 'Current Month',
        filters: {
          tag_and: [],
          tag_not: [],
          search_expression: '',
          alert: false,
          viewed: false,
          before: new Date().toISOString(),
          after: new Date(new Date().setDate(1)).toISOString(),
        },
      },
    ],
    currentFilter: {
      tag_and: [],
      tag_not: [],
      search_expression: '',
      alert: false,
      viewed: false,
      before: '',
      after: '',
    },
  },
  getters: {
    uri: state => `${(process.env.VUE_APP_SERVER_URL.startsWith('http') ? '' : 'http://')}${process.env.VUE_APP_SERVER_URL}${state.apiPath}`,
    // uri: state => `http://localhost:3000${state.apiPath}`,
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
    setRobotFilters(state, robot) {
      if (robot === 'all') {
        state.robotFilter = state.robots;
      } else {
        for (const r of state.robots) {
          if (r.id === robot) {
            state.robotFilter = [r];
          }
        }
      }
    },
    setEventFilters(state, filters) {
      const f = {
        tag_and: (filters.includeTags ? filters.includeTags : []),
        tag_not: (filters.excludeTags ? filters.excludeTags : []),
        search_expression: (filters.textSearch ? filters.textSearch : ''),
        alert: !!filters.other.notify,
        viewed: !!filters.other.onlyNew,
        before: (filters.beforeDate ? new Date(filters.beforeDate).toISOString() : ''),
        after: (filters.afterDate ? new Date(filters.afterDate).toISOString() : ''),
      };
      state.currentFilter = f;
    },
  },
  actions: {
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
            commit('setRobotFilters', 'all');
            dispatch('queryEvents', { filters: state.defaultFilter })
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
    filterEvents({ state, commit, dispatch }) {
      commit('resetEvents');
      commit('queryStarted');
      dispatch('queryEvents', { filters: state.currentFilter })
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
      console.log('Querying Patrols');
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
      console.log('Querying Schedules');
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
    queryEvents({ state, commit, dispatch }, filters) {
      console.log('Querying Events');
      commit('resetEvents');
      const robots = state.robotFilter;
      const options = {
        robots,
        index: 0,
        filters: filters.filters,
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
    saveSchedule({ getters }, schedule) {
      const req = {
        uri: `${getters.uri}/robots/${schedule.obj.robot}/schedules${schedule.id ? `/${schedule.id}` : ''}`,
        method: (schedule.id ? 'PUT' : 'POST'),
        headers: {
          'User-Agent': 'Request-Promise',
        },
        json: true,
        body: schedule.obj,
      };
      request(req)
        .then((result) => {
          console.log(result);
        }).catch((err) => {
          console.log(err);
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
          commit('fillWaypointList', wp, { root: true });
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
  },
};
