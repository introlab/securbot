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
const std_msgs = rosnodejs.require('std_msgs').msg;
const path = require('path');

/**
 * Keep a global reference of the window object so it doesn't automatically close.
 * @type {BrowserWindow}
 */
let win;

/**
 * Hub event emitter.
 * @type {events.EventEmitter}
 */
const hub = new events.EventEmitter();

/**
 * Interface between the msg data in ipc and hub
 */
ipcMain.on('msg', (event, arg) => {
  hub.emit('msg', arg);
});

/**
 * Create the window, loads the html into it and set events.
 * @function createWindow
 *
 * @fires rosdata
 */
function createWindow() {
  /**
   * Create the browser window.
   */
  win = new BrowserWindow({
    width: 800,
    height: 600,
    show: true,
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
   * Send data from ROS to the server.
   * @event rosdata
   * @type {object}
   * @property {String} data - Data coming from ROS to be sent to the server.
   */
  hub.on('rosdata', (data) => {
    win.webContents.send('rosdata', data);
  });

  /**
   * Emitted when the window is closed, remove listener and remove the window.
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
   */
  app.on('ready', createWindow);

  /**
   * Quit when all windows are closed.
   */
  app.on('window-all-closed', () => {
    if (process.platform !== 'darwin') {
      app.quit();
    }
  });

  /**
   * Create the window when activated
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
 *
 * @fires data
 * @fires patrol-plan
 * @fires msg
 * @fires parameters_response
 */
function startNode() {
  rosnodejs.initNode('/electron_webrtc').then(async (nodeHandle) => {
    /**
     * Fires after receiving data from ROS
     * @event data
     * @type {object}
     * @property {String} data - Data received from ROS
     */
    nodeHandle.subscribe('toElectron', std_msgs.String, (data) => {
      hub.emit('data', data);
    });

    /** Advertise the fromElectron Node  */
    const publisher = nodeHandle.advertise('fromElectron', std_msgs.String);

    /**
     * After receiving teleop command from server, publish it to the fromElectron Node
     * @event msg
     * @type {object}
     * @property {String} data - JSON string of the teleoperation command
     */
    hub.on('msg', (data) => {
      publisher.publish({ data });
    });

    /** advertise the operatorNavGoal node */
    const patrolPublisher = nodeHandle.advertise('/electron/patrol', std_msgs.String);
    /**
     * After receiving a patrol from server, publish it to the operatorNavGoal Node
     * @event patrol-plan
     * @type {object}
     * @property {String} patrolJsonString - JSON object containing the patrol.
     */
    ipcMain.on('patrol-plan', (event, patrolJsonString) => {
      patrolPublisher.publish({ data: patrolJsonString });
    });
  });
}

/**
 * Calls to start the application
 */
startApp();
startNode();
