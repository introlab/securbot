/**
 * This is the main script of the application.
 * @module Main
 * @author Edouard Denomme <>
 * @author Cedric Godin <>
 * @author Edouard Legare <>
 */


const { app, BrowserWindow, ipcMain } = require('electron');
const events = require('events');
const rosnodejs = require('rosnodejs');
// eslint-disable-next-line camelcase
const std_msgs = rosnodejs.require('std_msgs').msg;
const path = require('path');

/**
 * Keep a global reference of the window object so it doesn't automatically.
 * @type {BrowserWindow}
 */
let win;

/**
 * Hub event emitter.
 * @type {events.EventEmitter}
 */
const hub = new events.EventEmitter();

/**
 * Event emitter for ROS Node to app communication.
 * @event msg
 * @type {String}
 */
ipcMain.on('msg', (event, arg) => {
  hub.emit('msg', arg);
});

/**
 * Create the window, loads the html into it and set events.
 * @function createWindow
 */
function createWindow() {
  /**
   * Create the browser window.
   */
  win = new BrowserWindow({
    width: 800,
    height: 600,
    show: false,
    webPreferences: {
      nodeIntegration: false,
      preload: path.join(__dirname, 'ipcInit.js'),
    },
  });

  /**
   * Loads the html into the browser.
   */
  win.loadFile('index.html');

  /**
   *  Open the DevTools.
   */
  win.webContents.openDevTools();

  /**
   * Send data from ROS data hub.
   * @method
   * @param {String} data - data being send
   * @listens hub.rosdata
   */
  hub.on('rosdata', (data) => {
    win.webContents.send('rosdata', data);
  });

  /**
   * Emitted when the window is closed, remove listener and remove the window.
   * @method
   * @listens win.onclosed
   */
  win.on('closed', () => {
    // Dereference the window object, usually you would store windows
    // in an array if your app supports multi windows, this is the time
    // when you should delete the corresponding element.
    hub.removeAllListeners('rosdata');
    win = null;
  });
}

/**
 * Start the electron application
 * @function startApp
 */
function startApp() {
  /**
   * Create the window on event.
   * @listens app.ready
   */
  app.on('ready', createWindow);

  /**
   * Quit when all windows are closed.
   * @listens app.window.all.onclosed
   */
  app.on('window-all-closed', () => {
    if (process.platform !== 'darwin') {
      app.quit();
    }
  });

  /**
   * Create the window when activated
   * @listens app.activate
   */
  app.on('activate', () => {
    if (win === null) {
      createWindow();
    }
  });
}

/**
 * Start all the necessary ROS nodes
 * @function startNode
 */
function startNode() {
  rosnodejs.initNode('/electron_webrtc').then(async (nodeHandle) => {
    let webRtcServerUrl;
    // let videoDeviceLabel;
    try {
      webRtcServerUrl = await nodeHandle.getParam('/electron_webrtc/webrtc_server_url');
      // var videoDeviceLabel = await nodeHandle.getParam('/electron_webrtc/video_device_label')
    } catch (e) {
      console.error('Failed to retreive parameters');
      app.quit();
    }
    const parameters = { webRtcServerUrl }; // videoDeviceLabel
    console.log(parameters);

    if (win) { win.webContents.send('parameters_response', parameters); }

    ipcMain.on('parameters_request', (event) => {
      event.sender.send('parameters_response', parameters);
    });

    nodeHandle.subscribe('toElectron', std_msgs.String, (data) => {
      hub.emit('data', data);
    });

    const publisher = nodeHandle.advertise('fromElectron', std_msgs.String);
    hub.on('msg', (data) => {
      publisher.publish({ data });
    });

    // Navigation goal topic
    const goalPublisher = nodeHandle.advertise('operatorNavGoal', std_msgs.String);
    ipcMain.on('goal', (event, goalJsonString) => {
      goalPublisher.publish({ data: goalJsonString });
    });
  });
}

startApp();
startNode();
