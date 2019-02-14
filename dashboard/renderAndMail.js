const dModule = require('./dashboard.js');
const mail = require('./mail.js');

// Loading settings
try{
    var settings = require('./settings.json');
}catch(e){
    console.log('Error: Make sure to create the settings.json file');
    process.exit(1);
}

dModule.saveDashboard(settings).then(()=>{
    mail.send(settings);
});
