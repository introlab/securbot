const express = require('express')
const mongoose = require('mongoose')
const bodyParser = require('body-parser')
const paramSelector = require('request-param')



// REST API environment variables
const API_PORT = process.env.API_PORT || 8080


// MongoDB environment variables
const DB_HOST = process.env.DB_HOST || 'localhost'
const DB_PORT = process.env.DB_PORT || '27017'
const DB_URI = `mongodb://${DB_HOST}:${DB_PORT}/securbot`




// Database connection
console.log(`Attempting connection to ${DB_URI}`)
mongoose.Promise = global.Promise
// Deprecated methods disabled
mongoose.connect(DB_URI, {
    useNewUrlParser: true,
    useUnifiedTopology: true,
    useFindAndModify: false,
    useCreateIndex: true
});





// REST API server
console.log('Starting REST server on port: ' + API_PORT)
app = express()

app.use(bodyParser.urlencoded({ extended: true }))
app.use(bodyParser.json())

app.use(paramSelector({ order: ['query', 'body', 'params'] }))


// API routes
const routeRobots = require('./api/routes/robots')
routeRobots(app)



app.listen(API_PORT)
