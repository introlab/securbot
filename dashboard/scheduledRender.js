const dModule = require('./src/dashboard.js');
const schedule = require('node-schedule');
const mail = require('./src/mail.js');

// Loading settings
try{
    var settings = require('./settings.json');
}catch(e){
    console.log('Error: Make sure to create the settings.json file');
    process.exit(1);
}

// Update dashboards regularly
schedule.scheduleJob(settings.cron_schedule, ()=>{
    dModule.saveDashboard(settings);
});

// Send updated dashboards
schedule.scheduleJob(settings.cron_email, ()=>{
    mail.send(settings);
});
