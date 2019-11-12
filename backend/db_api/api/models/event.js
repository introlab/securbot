const mongoose = require ('mongoose');
const ObjectId = mongoose.Schema.Types.ObjectId;
const CoordinateSchema = require('./coordinate');



const EventSchema = new mongoose.Schema({
    robot: {
        type: ObjectId,
        required: true
    },
    object: {
        type: String,
        required: true
    },
    context: String,
    description_text: String,
    time: {
        type: Date,
        required: true
    },
    coordinate: CoordinateSchema,
    tags: [String],
    alert: {
        type: Boolean,
        default: false
    },
    viewed: {
        type: Boolean,
        default: false
    }
});

// Index for text searches
EventSchema.index( {
    object: 'text',
    context: 'text',
    description_text: 'text',
    tags: 'text'
})

module.exports = mongoose.model('Event', EventSchema);
