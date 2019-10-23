const schedules = require('../controllers/schedules')

module.exports = function (app) {

    app.route('/schedules')
        .get(schedules.list)
        .post(schedules.create)

    app.route('/schedules/:scheduleId')
        .put(schedules.update)
        .get(schedules.read)
}
