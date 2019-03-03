const puppeteer = require('puppeteer');
const datediff = require('date-diff');


// Genrate list total time spent
module.exports.getWorklog = async function (settings)
{
    // Opening chromium
    const browser = await puppeteer.launch({
        args: [ '--no-sandbox', '--disable-setuid-sandbox' ] });
    const page = await browser.newPage();
    console.log('Chromium started');

    console.log('Pulling worklog');

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
    try {
        await page.waitForNavigation({
            timeout: 10000,
            waitUntil: 'networkidle2'
        });
    } catch( e ) {};

    console.log('Navigating to Worklog');
    await page.goto(settings.worklogUrl, {
        timeout: 40000,
        waitUntil: 'networkidle0'
    });

    // Fetching entries in page
    var entriesList = await page.$$('.sc-hORach.kapOQE');
    entriesList.pop(); // removing total row

    var record = [];

    entriesList.forEach((line)=>{
        var nameP = line.$eval(
            '.sc-bMVAic.bKTmPP>span>span>span',
            element => element.innerText.trim());
        var timeP = line.$$eval(
            '.total_cell>div>div>div.cell_component',
            elementList => parseFloat(elementList[0].innerText.replace('h','').trim()));

        record.push(new Promise( async(resolve, reject) => {
            try{resolve({name: await nameP, time: await timeP})}
            catch(e){reject(e)};
        }));
    });

    record = await Promise.all(record);

    // Computing average time per 7 days according to start date
    var projectDays = new datediff(new Date(), new Date(settings.startDate)).days();
    record.forEach((entry) =>
        {entry.mean = Math.round( 100 * entry.time * 7.0 / projectDays)/100;});

    browser.close();

    return record;
}


// Adds the means in the browser context
module.exports.addMeans= function (records){
    let gadget = document.getElementById("iframe-gadget")
    let lines = gadget.contentDocument.getElementsByClassName('sc-hORach kapOQE');

    for(let container of lines) {
        var name = '';
        try{
            name = container.querySelector('.sc-bMVAic.bKTmPP>span>span>span').innerText.trim();
        } catch(e){};
        if (name == '') return;

        records.find((member)=>{
            if (name ==  member.name)
            {
                // Change the field with total
                container.querySelectorAll('.total_cell>div>div')[0].innerHTML =
                    `Avg(${member.mean}h) Total`;

                return true;
            }

            return false;
        })
    }
}
