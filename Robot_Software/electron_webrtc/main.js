const { app, BrowserWindow, ipcMain } = require('electron')
const events = require('events')

const rosnodejs = require('rosnodejs')
const std_msgs = rosnodejs.require('std_msgs').msg

const path = require('path')


// Keep a global reference of the window object, if you don't, the window will
// be closed automatically when the JavaScript object is garbage collected.
let win

// Event emitter for ROS Node to app communication
let hub = new events.EventEmitter()
ipcMain.on('msg', (event, arg) => {
    hub.emit('msg', arg)
})

function createWindow () {
  // Create the browser window.
  win = new BrowserWindow({ width: 800,
                            height: 600,
                            show: false,
                            webPreferences: {nodeIntegration: false,
                                             preload: path.join(__dirname, 'ipcInit.js')}})

  // and load the index.html of the app.
  win.loadFile('index.html')

  // Open the DevTools.
  win.webContents.openDevTools()

  // Send data from ROS data hub
  hub.on('rosdata', (data) => {
    win.webContents.send('rosdata', data)
  })

  // Emitted when the window is closed.
  win.on('closed', () => {
    // Dereference the window object, usually you would store windows
    // in an array if your app supports multi windows, this is the time
    // when you should delete the corresponding element.
    hub.removeAllListeners('rosdata')
    win = null
  })
}

function startApp() {

  // This method will be called when Electron has finished
  // initialization and is ready to create browser windows.
  // Some APIs can only be used after this event occurs.
  app.on('ready', createWindow)

  // Quit when all windows are closed.
  app.on('window-all-closed', () => {
    // On macOS it is common for applications and their menu bar
    // to stay active until the user quits explicitly with Cmd + Q
    if (process.platform !== 'darwin') {
      app.quit()
    }
  })

  app.on('activate', () => {
    // On macOS it's common to re-create a window in the app when the
    // dock icon is clicked and there are no other windows open.
    if (win === null) {
      createWindow()
    }
  })
}

// In this file you can include the rest of your app's specific main process
// code. You can also put them in separate files and require them here.

function startNode() {
  rosnodejs.initNode('/electron_webrtc').then( async (nodeHandle) => {

    try {
      var webRtcServerUrl = await nodeHandle.getParam('/electron_webrtc/webrtc_server_url')
      var videoDeviceLabel = await nodeHandle.getParam('/electron_webrtc/video_device_label')
    } catch (e) {
      console.error('Failed to retreive parameters')
      app.quit()
    }
    var parameters = { videoDeviceLabel, webRtcServerUrl }
    console.log(parameters);

    if (win)
        win.webContents.send('parameters_response', parameters)

    ipcMain.on('parameters_request', (event) => {
      event.sender.send('parameters_response', parameters)
    })

    let subscriber = nodeHandle.subscribe('toElectron', std_msgs.String, (data) => {
      hub.emit('data', data)
    })

    let publisher = nodeHandle.advertise('fromElectron', std_msgs.String)
    publisher.publish({ data: 'Hello!' })
    hub.on('msg', data => {
        console.log(data)
        publisher.publish({ data: data })
    })
  })
}

startApp()
startNode()
