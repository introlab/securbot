const { check, param, checkSchema } = require('express-validator')
const filterSchema = require('../validators/event_filter')


module.exports = function (app) {
    const events = require('../controllers/events')

    app.route('/events')
        .get(
            checkSchema(filterSchema),
            events.list)

        .post(events.publish)

    app.route('/events/:eventId')
        .get(events.read)
        .patch(
            param('eventId').isMongoId().withMessage('Not a valid event ID'),
            check('viewed').isBoolean().withMessage('Must be boolean')
                .toBoolean().custom((value)=>value).withMessage('Must be true'),
            events.markAsRead)

    app.get('/event_tags', events.listTags)
}
