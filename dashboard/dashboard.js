const puppeteer = require('puppeteer');
const bigpic = require('./BigPic/node-big-pic.js');
const pdf = require('./pdf.js');

module.exports.saveDashboard = async function (settings){
    console.log('Starting Dashboard render');
    await renderDashboard(settings);

    console.log('Starting PDF conversion');
    pdf.generatePDF(settings.output, settings.outputPDF);
    pdf.generateLandscapePDF(settings.output, settings.outputLandscapePDF);

    console.log('Finished Dashboard export');
}

async function renderDashboard(settings){
    const browser = await puppeteer.launch({
        args: [
            '--no-sandbox',
            '--disable-setuid-sandbox'
        ],
        defaultViewport: {
            width: settings.screenWidth,
            height: 1080,
            deviceScaleFactor: settings.deviceScalingFactor
        }
    });
    const page = await browser.newPage();
    console.log('Chromium started');

    // Load Page
    console.log('Loading page...');
    await page.goto(settings.url);
    console.log('Page loaded');

    // Log into Jira
    console.log('Filling login ...');
    await page.type('#login-form-username', settings.user);
    await page.type('#login-form-password', settings.passwd);
    await page.click('#login');
    console.log('Submitting login');

    console.log('Waiting for dashboard ...');
    await page.waitForNavigation({timeout: 120000});

    console.log('Waiting for Worklogs ...');
    var worklogFrame = page.frames().find(frame => frame.url().includes('worklog'));
    await worklogFrame.waitForSelector('#worklogs_main>div>div.main-content', {timeout: 60000});

    console.log('Acquiring dashboard')
    var dashboard = await page.$('#dashboard-content');

    // Apply the BigPic extension
    console.log('Adjusting picture size')
    await page.evaluate(bigpic)

    console.log('Printing dashboard')
    var image = await dashboard.screenshot({path: settings.output});
    browser.close();

    return image;
}
