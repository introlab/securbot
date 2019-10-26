const mongoose = require ('mongoose');


const RobotSchema = new mongoose.Schema({
    name: {
        type: String,
        required: true
    },
    platform: {
        OS: String,
        ROS: String,
        CPU: String,
        GPU: String,
        Batteries: String,
        Frame: String
    }
})

module.exports = mongoose.model('Robot', RobotSchema);
