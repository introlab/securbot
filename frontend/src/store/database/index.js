/* eslint-disable no-underscore-dangle */
import request from 'request-promise';

export default {
  namespaced: true,
  state: {
    apiPath: '/db',
    queryingDB: false,
    errorDuringQuery: false,
    events: [],
    headers: ['Robot', 'Object', 'Context', 'Description', 'Tags', 'DateTime', 'Image'],
    eventList: [],
    robots: [],
    robotFilter: [],
    tagList: ['red', 'yellow', 'blue', 'green', 'a', 'b', 'c'],
    defaultFilter: {
      tag_and: [],
      tag_not: [],
      search_expression: '',
      alert: '',
      viewed: '',
      before: new Date().toISOString(),
      after: new Date(new Date().getTime() - (7 * 24 * 60 * 60 * 1000)).toISOString(),
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
        }
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
    uri: state => `http://${process.env.VUE_APP_SERVER_URL}${state.apiPath}`,
    // uri: state => `http://localhost:3000${state.apiPath}`,
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
    resetEvent(state) {
      state.events.splice(0, state.events.length - 1);
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
        alert: (filters.other.notify ? true : ''),
        viewed: (filters.other.onlyNew ? true : ''),
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
      commit('queryStarted');
      dispatch('queryRobots')
        .then(() => {
          commit('setRobotFilters', 'all');
          dispatch('queryEvents', { filters: state.defaultFilter })
            .then(() => {
              commit('queryFinished');
            }).catch((err) => {
              console.log(err);
              commit('errorDuringQuery');
              commit('queryFinished');
            });
        }).catch((err) => {
          console.log(err);
          commit('errorDuringQuery');
          commit('queryFinished');
        });
    },
    filterEvents({ state, commit, dispatch }) {
      commit('resetEvents');
      commit('queryStarted');
      console.log(state.currentFilter);
      console.log(state.events);
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
    queryPatrols({ state, getters, commit, dispatch }, index) {
      let _index = (!index ? 0 : index);
      return new Promise((resolve, reject) => {
        // All patrols for each robot have been queried
        if (_index >= state.robots.length) {
          resolve();
        }
        // Query robot
        const robot = state.robots[_index];
        const req = {
          uri: `${getters.uri}/robots/${robot.id}/patrols`,
          headers: {
            'User-Agent': 'Request-Promise',
          },
          json: true,
        };
        // Request
        request(req)
          .then((result) => {
            result.forEach((patrol) => {
              commit('addPatrol', { name: patrol.name, robot: patrol.robot, id: patrol._id });
            });
            _index += 1;
            return dispatch('queryPatrols', _index)
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
    queryPatrol({ getters, rootCommit }, patrol) {
      const req = {
        uri: `${getters.uri}/robots/${patrol.robot}/patrols/${patrol.id}`,
        headers: {
          'User-Agent': 'Request-Promise',
        },
        json: true,
      };
      return new Promise((resolve, reject) => {
        request(req)
          .then((result) => {
            const waypointList = result.waypoints;
            rootCommit('fillWaypointList', waypointList);
            resolve();
          }).catch((err) => {
            reject(err);
          });
      });
    },
    querySchedules() {
    },
    // eslint-disable-next-line object-curly-newline
    queryEvents({ state, dispatch }, filters) {
      const robots = state.robotFilter;
      const options = {
        robots,
        index: 0,
        filters: filters.filters,
      };
      console.log(options);
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
          body: options.filters,
          headers: {
            'User-Agent': 'Request-Promise',
          },
          json: true,
        };
        console.log(req);
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
  },
};
