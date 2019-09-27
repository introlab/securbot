const mongoose = require ('mongoose');
const ObjectId = mongoose.Schema.Types.ObjectId;



const ScheduleSchema = new mongoose.Schema({
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
    patrol: {
        type: ObjectId,
        required: true
    },
    cron: {
        type: String,
        required: true
    },
    timeout_s: Number,
    repetitions: Number,
    enabled: {
        type: Boolean,
        required: true
    }
});


module.exports = mongoose.model('Schema', ScheduleSchema);
