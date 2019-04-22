/**
 * Initialisation of the ipc
 * @module ipcInit
 * @author Cedric Godin <>
 */

const { ipcRenderer } = require('electron');

process.once('loaded', () => {
  global.ipc = ipcRenderer;
  global.hello = 'hello';
});
