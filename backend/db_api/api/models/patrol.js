const mongoose = require ('mongoose');
const ObjectId = mongoose.Schema.Types.ObjectId;


const CoordinateSchema = new mongoose.Schema({
    x: { type: Number, required: true },
    y: { type: Number, required: true },
    yaw: {
        type: Number,
        required: true,
        min: [-360, 'Yaw must be bound between [-360, 360]'],
        max: [360, 'Yaw must be bound between [-360, 360]'],
    },
    _id: false
})


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
