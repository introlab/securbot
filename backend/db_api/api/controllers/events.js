const mongoose = require('mongoose')
const Model = require('../models/event')
const { matchedData, validationResult } = require('express-validator')



exports.list = function (req, res) {

    // Input error check
    const errors = validationResult(req)
    if (!errors.isEmpty()) {
        res.status(400).json({ errors: errors.array() })
        return
    }

    // Input results
    filters = matchedData(req)


    // Building query
    query = {
        robot: req.robot
    }

    if (filters.tag_and)
        query.tags = { $all: filters.tag_and }
    if (filters.tag_or) {
        query.tags = query.tags || {}
        query.tags.$in = filters.tag_or
    }
    if (filters.tag_not) {
        query.tags = query.tags || {}
        query.tags.$nin = filters.tag_not
    }

    if (filters.search_expression)
        query.$text = { $search: filters.search_expression }

    if ('alert' in filters)
        query.alert = filters.alert
    if ('viewed' in filters)
        query.viewed = filters.viewed

    if (filters.before)
        query.time = { $lte: filters.before }
    if (filters.after) {
        query.time = query.time || {}
        query.time.$gte = filters.after
    }


    Model.find(query)
        .sort('-time')
        .select(filters.minimized ? 'object time alert viewed' : '')
        .exec((err, documents) => {
            if (err)
                res.status(500).send(err)

            if (! documents)
                res.status(404)

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

    // Input error check
    const errors = validationResult(req)
    if (!errors.isEmpty()) {
        res.status(400).json({ errors: errors.array() })
        return
    }

    Model.findOneAndUpdate({_id: req.params.eventId}, { viewed: true }, {new: true}, (err, savedDocument) => {
        if (err)
            res.status(500).send(err)

        if (! savedDocument)
            res.status(404)

        res.json(savedDocument)
    })
}

exports.read = function (req, res) {
    Model.findById(req.params.eventId, (err, document) => {
        if (err)
            res.status(500).send(err)

        if (! document)
            res.status(404)

        res.json(document)
    })
}

exports.listTags = function (req, res) {
    Model.distinct('tags', { robot: req.robot }, (err, tagList) => {
        if (err)
            res.status(500).send(err)

        if (! tagList)
            res.status(404)

        res.json(tagList)
    })
}
