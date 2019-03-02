// Load required modules
var http    = require("http");              // http server core module
var express = require("express");           // web framework external module
var socketIo = require("socket.io");        // web socket external module
var easyrtc = require("easyrtc");           // EasyRTC internal module
var path = require('path');

// Set process name
process.title = "securBot-EasyRTC-Server";

// Setup and configure Express http server. Expect a subfolder called "static" to be the web root.
var app = express();
app.use(express.static(path.join(__dirname, 'public')));

// Start Express http server on port 8080
var webServer = http.createServer(app);

// Start Socket.io so it attaches itself to Express server
var socketServer = socketIo.listen(webServer, {"log level":1});

easyrtc.setOption("logLevel", "debug");

// Start EasyRTC server
var rtc = easyrtc.listen(app, socketServer);

// Listen on port 8080
webServer.listen(8081, function () {
    console.log('listening on http://localhost:8080');
});
