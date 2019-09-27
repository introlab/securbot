const mongoose = require('mongoose')
const Model = require('../models/robot')


exports.list = function (req, res) {
    Model.find({}, (err, documents) => {
        if (err)
            res.status(500).send(err)
        res.json(documents)
    })
}

exports.create = function (req, res) {
    let newDocument = new Model(req.body)
    newDocument.save((err, savedDocument) => {
        if (err)
            res.status(500).send(err)
        res.json(savedDocument)
    })
}

exports.update = function (req, res) {
    Model.findOneAndUpdate({_id: req.params.robotId}, req.body, {new: true}, (err, savedDocument) => {
        if (err)
            res.status(500).send(err)
        res.json(savedDocument)
    })
}

exports.read = function (req, res) {
    Model.findById(req.params.robotId, (err, document) => {
        if (err)
            res.status(500).send(err)
        res.json(document)
    })
}
