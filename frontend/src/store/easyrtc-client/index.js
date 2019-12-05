import state from './states';
import getters from './getters';
import mutations from './mutations';
import actions from './actions';

/**
 * The easyrtc-client export.
 *
 * @exports
 * @module Store
 */
export default {
  namespaced: true,
  state,
  getters,
  mutations,
  actions,
};
