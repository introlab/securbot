# Electron Application

This package is the electron application used to interface the easyrtc server with ROS.

## Content
The package contains the following files:

* main.js : Main file of the application
* renderer.js : Code for the renderer
* ipcInit.js : Initialise the ipc
* index.html : The web page
* launch:
   * electron_webrtc.launch : The launch file of the application to make it a ROS node.
*  scripts
   * start_node.bash : script to start the electron browser.

## Commands
The following command will generate the documentation for the application:
```
$ npm run docs
```

Even though not recommended, you can start the electron with the following command:
```
$ npm run start
```
