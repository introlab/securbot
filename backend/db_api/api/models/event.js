const mongoose = require ('mongoose');
const ObjectId = mongoose.Schema.Types.ObjectId;
const CoordinateSchema = require('./coordinate');


const FileSchema = new mongoose.Schema({
    fieldname: String,
    originalname: String,
    encoding: String,
    mimetype: String,
    id: String,
    filename: String,
    size: Number,
    md5: String,
    uploadDate: String,
    contentType: String,
    _id: false
})


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
    files: [FileSchema],
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
