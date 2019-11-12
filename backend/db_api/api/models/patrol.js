const mongoose = require ('mongoose');
const ObjectId = mongoose.Schema.Types.ObjectId;
const CoordinateSchema = require('./coordinate');



const WaypointSchema = new mongoose.Schema({
    coordinate: {
        type: CoordinateSchema,
        required: true
    },
    hold_time_s: Number,
    _id: false
});

const PatrolSchema = new mongoose.Schema({
    robot: {
        type: ObjectId,
        required: true
    },
    name: {
        type: String,
        required: true
    },
    description_text: String,
    last_modified: {
        type: Date,
        default: Date.now
    },
    waypoints: {
        type: [WaypointSchema],
        required: true
    }
});

module.exports = mongoose.model('Patrol', PatrolSchema);
