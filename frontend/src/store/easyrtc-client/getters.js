export default {
  currentRobot: (state) => {
    const r = {
      name: '',
      id: {
        client: '',
        db: '',
      },
    };
    for (const cRobot of state.client.robotList) {
      for (const dbRobot of state.database.robots) {
        if (state.client.robotId === cRobot.robotId
          && state.client.robotName === dbRobot.name) {
          r.name = state.client.robotName;
          r.id.client = state.client.robotId;
          r.id.db = dbRobot.id;
        }
      }
    }
    return r;
  },
};
