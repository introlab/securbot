import Vue from 'vue';
import Vuex from 'vuex';
import client from './easyrtc-client';
import database from './database';

import state from './states';
import getters from './getters';
import mutations from './mutations';
import actions from './actions';

Vue.use(Vuex);

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
