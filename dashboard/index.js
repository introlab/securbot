const puppeteer = require('puppeteer-core');
const dModule = require('./dashboard.js');

// Loading settings
try{
    var settings = require('./settings.json');
}catch(e){
    console.log('Error: Make sure to create the settings.json file');
    process.exit(1);
}

dModule.getDashboard(settings);
