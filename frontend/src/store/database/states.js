/**
 * The database module states.
 *
 * @module Database
 * @exports
 */
export default {
  /**
   * The database api path (what follows the url: `url${apiPath}`), env variable.
   */
  apiPath: process.env.VUE_APP_DB_PATH,
  /**
   * The server url, env variable
   */
  serverURL: (
    process.env.VUE_APP_DEV_DB_URL
      ? process.env.VUE_APP_DEV_DB_URL
      : process.env.VUE_APP_SERVER_URL
  ),
  /**
   * Is the app querying the database right now.
   */
  queryingDB: false,
  /**
   * If true, the last query was rejected.
   */
  errorDuringQuery: false,
  /**
   * The list of events obtained from the database.
   */
  events: [],
  /**
   * The url of the event image.
   */
  eventImageURL: '',
  /**
   * List of robot obtained fromthe database.
   */
  robots: [],
  /**
   * When querying database, only the robots included in this array will be queried.
   */
  robotFilter: [],
  /**
   * Tag list for filtering. Currently fill with placeholders, needs a way to get all tags from
   * the database after inplementing them.
   */
  tagList: ['blue', 'yellow', 'red', 'green'],
  /**
   * The fefault filter used by the frontend for the initial query.
   */
  defaultFilter: {
    after: 7 * 24 * 60 * 60 * 1000,
    viewed: false,
    alert: false,
  },
  /**
   * A list of already programmed filters. Because of an issue with the database, some might not
   * work correctly even though all parameters are correct.
   */
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
  /**
   * The customized event filter used by the event query.
   */
  eventFilter: {},
};
