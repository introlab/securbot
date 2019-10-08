const puppeteer = require('puppeteer');
const bigpic = require('./BigPic/node-big-pic.js');
const pdf = require('./pdf.js');
const worklog = require('./worklog.js');
const burndown = require('./burndown.js');
const sleep = require('await-sleep');

module.exports.saveDashboard = async function (settings){

    console.group('Extracting Worklog')
    try {
        var worklogP = await worklog.getWorklog(settings);
    } catch (e) {
        console.log('Failed to retreive worklog');
        console.error(e);
        console.groupEnd();
        console.log('Terminating dashboard export');
        throw "Failed to compute worklog";
    }
    console.groupEnd();

    console.group('Starting Dashboard render');
    try {
        await renderDashboard(settings, worklogP);
    } catch (e) {
        console.log('Failed to render dashboard');
        console.error(e);
        console.groupEnd();
        console.log('Terminating dashboard export');
        throw "Failed to render";
    }
    console.groupEnd();

    console.group('Starting PDF conversion');
    try {
        pdf.generatePDF(settings.output, settings.outputPDF);
        await pdf.generateLandscapePDF(settings.output, settings.outputLandscapePDF);
    } catch (e) {
        console.log('Failed to export to PDF');
        console.error(e);
        console.groupEnd();
        console.log('Terminating dashboard export');
        return;
    }
    console.groupEnd();

    console.log('Finished Dashboard export');
}

async function renderDashboard(settings, worklogP){
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

    try {
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
        await worklogFrame.waitForSelector('table.sc-fAjcbJ.hduqMu', {timeout: 60000});
        await sleep(1000);

        // Apply the BigPic extension
        console.log('Adjusting picture size')
        await page.evaluate(bigpic)
        await sleep(500);

        // Clearing burndown legend from graph
        console.log('Adjusting burndown legend');
        try {
            await page.evaluate(burndown.clearLegend);
        } catch (e) {
            console.log('Failed to adjust burndown legend');
            console.error(e);
        }

        // Adding the means to the table
        console.log('Adding worklog means');
        try {
            await page.evaluate(worklog.addMeans, await worklogP);
        } catch (e) {
            console.log('Failed to add means')
            console.error(e);
        }

        console.log('Acquiring dashboard')
        var dashboard = await page.$('#dashboard-content');

        console.log('Printing dashboard')
        var image = await dashboard.screenshot({path: settings.output});
    } catch (e) {
        browser.close();
        throw e;
    }

    browser.close();

    return image;
}
