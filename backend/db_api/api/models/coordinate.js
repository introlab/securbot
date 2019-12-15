const mongoose = require ('mongoose');


module.exports = new mongoose.Schema({
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
