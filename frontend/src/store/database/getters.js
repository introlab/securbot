/**
 * The database module getters.
 *
 * @module Database
 * @exports
 */
export default {
  /**
   * Return the complete uri for the database (protocole://url/apiPath).
   */
  uri: state => `${(state.serverURL.startsWith('http') ? '' : 'http://')}${state.serverURL}${state.apiPath}`,
};
