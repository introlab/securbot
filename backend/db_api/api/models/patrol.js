const mongoose = require ('mongoose');
const ObjectId = mongoose.Schema.Types.ObjectId;



const WaypointSchema = new mongoose.Schema({
    quaternion: {
        a: Number,
        b: Number,
        c: Number,
        d: Number,
        required: true
    },
    hold_time_s: Number
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
        type: String,
        default: Date.now
    },
    waypoints: {
        type: [WaypointSchema],
        required: true
    }
});

module.exports = mongoose.model('Patrol', PatrolSchema);
