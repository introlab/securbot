export default {
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
