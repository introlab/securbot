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
    tagList: ['red', 'yellow', 'blue', 'green', 'a', 'b', 'c'],
  },
  getters: {
    uri: state => `http://${process.env.VUE_APP_SERVER_URL}${state.apiPath}`,
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
  },
  actions: {
    initLocalData({ commit, dispatch }) {
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
          dispatch('queryEvents')
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
    queryEvents({ state, getters, commit, dispatch }, index) {
      let _index = (!index ? 0 : index);
      return new Promise((resolve, reject) => {
        // All events for each robots have been queried
        if (_index >= state.robots.length) {
          resolve();
        } else {
          // Query robot
          const robot = state.robots[_index];
          const req = {
            uri: `${getters.uri}/robots/${robot.id}/events`,
            headers: {
              'User-Agent': 'Request-Promise',
            },
            json: true,
          };
          // Request
          request(req)
            .then((result) => {
              result.forEach((event) => {
                commit('addEvent', event);
              });
              _index += 1;
              return dispatch('queryEvents', _index)
                .then(() => {
                  resolve();
                }).catch((err) => {
                  reject(err);
                });
            }).catch((err) => {
              reject(err);
            });
        }
      });
    },
  },
};
