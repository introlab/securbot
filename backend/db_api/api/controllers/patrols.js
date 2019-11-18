const mongoose = require('mongoose')
const Model = require('../models/patrol')


exports.list = function (req, res) {
    Model.find({ robot: req.robot }, 'name', (err, documents) => {
        if (err)
            res.status(500).send(err)

        if (! documents)
            res.status(404)

        res.json(documents)
    })
}

exports.create = function (req, res) {
    let newDocument = new Model(req.body)

    if (newDocument.robot == undefined)
        newDocument.robot = req.robot
    else if (newDocument.robot != req.robot)
        return res.status(409).send(`Conflicting "robot" id between:\npath : "${req.robot}"\nbody : "${newDocument.robot}"`)

    newDocument.save((err, savedDocument) => {
        if (err)
            res.status(500).send(err)

        res.json(savedDocument)
    })
}

exports.update = function (req, res) {
    Model.findOneAndUpdate({_id: req.params.patrolId}, req.body, {new: true}, (err, savedDocument) => {
        if (err)
            res.status(500).send(err)

        if (! savedDocument)
            res.status(404)

        res.json(savedDocument)
    })
}

exports.read = function (req, res) {
    Model.findById(req.params.patrolId, (err, document) => {
        if (err)
            res.status(500).send(err)

        if (! document)
            res.status(404)

        res.json(document)
    })
}

exports.remove = function (req, res) {
    Model.findByIdAndDelete(req.params.patrolId, (err, document) => {
        if (err)
            res.status(500).send(err)

        if (! document)
            res.status(404)

        res.json(document)
    })
}
