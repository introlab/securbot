const { check, param, checkSchema } = require('express-validator')

module.exports = function (app) {
    const files = require('../controllers/files')

    app.post('/files', files.upload)
    app.get(
        '/files/:fileId',
        param('fileId').isMongoId().withMessage('Not a valid event ID'),
        files.download)
}

