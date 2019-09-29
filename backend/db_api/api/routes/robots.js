const robots = require('../controllers/robots')

module.exports = function (app) {

    app.route('/robots')
        .get(robots.list)
        .post(robots.create)

    app.route('/robots/:robotId')
        .put(robots.update)
        .get(robots.read)

    const router = require('express').Router()

    const routeEvents = require('./events')
    routeEvents(router)

    const routePatrols = require('./patrols')
    routePatrols(router)

    const routeSchedules = require('./schedules')
    routeSchedules(router)


    app.use('/robots/:robotId', (req, res, next) => {
        req.robot = req.params.robotId
        next()
    }, router)
}
