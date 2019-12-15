const dModule = require('./src/dashboard.js');

// Loading settings
try{
    var settings = require('./settings.json');
}catch(e){
    console.log('Error: Make sure to create the settings.json file');
    process.exit(1);
}

try {
    dModule.saveDashboard(settings);
}
catch (e) {
    process.exit(1);
}
