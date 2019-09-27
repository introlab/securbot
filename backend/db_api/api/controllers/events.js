const mongoose = require('mongoose')
const Model = require('../models/event')


exports.list = function (req, res) {
    Model.find({robot: req.robot}, (err, documents) => {
        if (err)
            res.status(500).send(err)
        res.json(documents)
    })
}

exports.publish = function (req, res) {
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

exports.markAsRead = function (req, res) {
    Model.findOneAndUpdate({_id: req.params.eventId}, req.body, {new: true}, (err, savedDocument) => {
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
