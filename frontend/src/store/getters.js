/**
 * The root store getters.
 *
 * @module Store
 * @exports
 */
export default {
  /**
   * Returns only the event of the connected robot.
   */
  eventsWaypoints: (state) => {
    const wpl = [];
    for (const event of state.database.events) {
      if (state.currentRobot.id.db === event.robot) {
        wpl.push(event);
      }
    }
    return wpl;
  },
};
