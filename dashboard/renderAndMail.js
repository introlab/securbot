const dModule = require('./src/dashboard.js');
const mail = require('./src/mail.js');

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
