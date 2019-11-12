export default {
  connectionState: {
    server: 'disconnected',
    robot: 'disconnected',
  },
  myId: '',
  robotId: '',
  cameraStream: undefined,
  mapStream: undefined,
  robotList: [],
  isDataChannelAvailable: false,
  robotStatus: {
    resources: {
      cpu: 0,
      mem: 0,
    },
    wifi: {
      rssi: 0,
      noise: 0,
    },
    intention: {
      strategies: [],
      desires: [],
    },
  },
};
