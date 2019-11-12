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
  hub.on('robot-status', (data) => {
    win.webContents.send('robot-status', data);
  });

  hub.on('map-size', (data) => {
    win.webContents.send('map-size', data);
  });

  /**
   * Emitted when the window is closed, remove listener and remove the window.
   */
  win.on('closed', () => {
    // Dereference the window object, usually you would store windows
    // in an array if your app supports multi windows, this is the time
    // when you should delete the corresponding element.
    hub.removeAllListeners('robot-status');
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
  rosnodejs.initNode('electron_webrtc').then(async (nodeHandle) => {
    /**
     * Fires after receiving data from ROS
     * @event data
     * @type {object}
     * @property {String} data - Data received from ROS
     */
    nodeHandle.subscribe('robot_status', std_msgs.String, (data) => {
      hub.emit('robot-status', data);
    });

    // Retrieves map size from parameter server
    let mapParam = {
      resolution: 0,
      width: 0,
      height: 0
    };
    nodeHandle.getParam('/map_image_generator/resolution')
    .then(resolution => {
      mapParam.resolution = resolution;
      return nodeHandle.getParam('/map_image_generator/width');
    })
    .then(width => {
      mapParam.width = width;
      return nodeHandle.getParam('/map_image_generator/height');
    })
    .then(height => {
      mapParam.height = height;
      hub.emit('map-size', {
        height: mapParam.resolution * mapParam.height,
        width: mapParam.resolution * mapParam.width
      });
    });

    /** Advertise the teleop topic  */
    const teleopPublisher = nodeHandle.advertise('teleop', std_msgs.String);
    /**
     * After receiving teleop command from server, publish it to the fromElectron Node
     * @type {object}
     * @property {String} joystickJsonString - JSON string of the teleoperation command
     */
    ipcMain.on('joystick-position', (_, joystickJsonString) => {
      teleopPublisher.publish({ data: joystickJsonString});
    });

    /** advertise the patrol topic */
    const patrolPublisher = nodeHandle.advertise('patrol', std_msgs.String);
    /**
     * After receiving a patrol from server, publish it to the patrol topic
     * @type {object}
     * @property {String} patrolJsonString - JSON object containing the patrol.
     */
    ipcMain.on('patrol-plan', (_, patrolJsonString) => {
      patrolPublisher.publish({ data: patrolJsonString });
    });

    /** Publish received goto to ROS */
    const gotoPublisher = nodeHandle.advertise('goto', std_msgs.String);
    ipcMain.on('goto', (_, gotoString) => {
      gotoPublisher.publish({ data: gotoString });
    });

    /** Publish received map zoom to ROS */
    const zoomPublisher = nodeHandle.advertise('map_zoom', std_msgs.String);
    ipcMain.on('changeMapZoom', (_, zoomString) => {
      zoomPublisher.publish({ data: zoomString });
    })
  });
}

/**
 * Calls to start the application
 */
startApp();
startNode();
