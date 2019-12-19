import state from './states';
import getters from './getters';
import mutations from './mutations';
import actions from './actions';

/**
 * The database client export.
 *
 * @exports
 * @module Database
 */
export default {
  namespaced: true,
  state,
  getters,
  mutations,
  actions,
};
