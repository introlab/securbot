/**
 * Initialisation of the ipc
 * @module ipcInit
 * @author Cedric Godin <>
 */

const { ipcRenderer } = require('electron');

process.once('loaded', () => {
  global.ipc = ipcRenderer;
  global.webrtcServerURL = process.env.SECURBOT_SERVER_URL;
  global.virtualDeviceNames = [process.env.VIRTUAL_DEVICE_CAMERA, process.env.VIRTUAL_DEVICE_MAP];
  global.robotName = process.env.ROBOT_NAME;
  global.hello = 'hello';
});
