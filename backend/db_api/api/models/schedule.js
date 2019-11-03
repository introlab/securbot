const mongoose = require ('mongoose');
const ObjectId = mongoose.Schema.Types.ObjectId;
const cron = require('cron-validator')



const ScheduleSchema = new mongoose.Schema({
    robot: {
        type: ObjectId,
        required: true
    },
    name: {
        type: String,
        required: true,
        trim: true
    },
    description_text: {
        type: String,
        trim: true
    },
    last_modified: {
        type: Date,
        default: Date.now
    },
    patrol: {
        type: ObjectId,
        required: true
    },
    cron: {
        type: String,
        required: true,
        trim: true,
        validate: {
            validator: cron.isValidCron,
            alias: true,
            seconds: false,
            propsParameter: true,
            message: 'Must be a valid Cron syntax'
        }
    },
    timeout_s: Number,
    repetitions: Number,
    enabled: {
        type: Boolean,
        required: true
    }
});


module.exports = mongoose.model('Schema', ScheduleSchema);
