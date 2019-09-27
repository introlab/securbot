module.exports = function (app) {
    const events = require('../controllers/events')


    app.route('/events')
        .get(events.list)
        .post(events.publish)

    app.route('/events/:eventId')
        .get(events.read)
        .patch(events.markAsRead)
}
