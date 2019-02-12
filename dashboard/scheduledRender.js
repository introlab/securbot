const dModule = require('./dashboard.js');
const schedule = require('node-schedule');

// Loading settings
try{
    var settings = require('./settings.json');
}catch(e){
    console.log('Error: Make sure to create the settings.json file');
    process.exit(1);
}

schedule.scheduleJob(settings.cron_schedule, ()=>{
    dModule.saveDashboard(settings);
})
