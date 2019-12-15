const mongoose = require('mongoose')
const multer = require('multer')
const storage = require('multer-gridfs-storage')
const { createModel } = require('mongoose-gridfs')
const { matchedData, validationResult } = require('express-validator')


const fileModel = storage({
    db: mongoose.connection,
});

const upload = multer({
    storage: fileModel
}).single('file')

// Default file model
const Model = createModel()



exports.download = function (req, res) {
    const errors = validationResult(req)
    if (!errors.isEmpty()) {
        res.status(400).json({ errors: errors.array() })
        return
    }

    Model.findOne({_id: req.params.fileId}, (err, file) => {
        if (err)
            res.status(500).json({success: false, err})
        else if(!file)
            res.status(404).json({success: false})
        else {

            let stream = Model.read(file)

            res.set('Content-Type', file.contentType)
            return stream.pipe(res)
        }
    })
}


exports.upload = function (req, res) {
    upload(req,res, (err) => {
        if (err || !req.file) {
            res.status(500).json({success: false, err})
            return
        }

        res.json({
            success: true,
            _id: req.file.id,
            file: req.file
        })
    })
}
