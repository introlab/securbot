import Vue from 'vue';
import Vuex from 'vuex';
import client from './easyrtc-client';
import database from './database';

import state from './states';
import getters from './getters';
import mutations from './mutations';
import actions from './actions';

Vue.use(Vuex);

Vue.config.devtools = true;

/**
 * The store component. This component is split into multiple files to make it easier to understand
 * and use. For Vuex documentation see [docs](https://vuex.vuejs.org/).
 * @module Store
 * @exports
 */
export default new Vuex.Store({
  modules: {
    client,
    database,
  },
  state,
  getters,
  mutations,
  actions,
});
