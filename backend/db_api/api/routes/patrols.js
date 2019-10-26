const patrols = require('../controllers/patrols')

module.exports = function (app) {

    app.route('/patrols')
        .get(patrols.list)
        .post(patrols.create)

    app.route('/patrols/:patrolId')
        .put(patrols.update)
        .get(patrols.read)
}
