# SecurBot dashboard

Export tools suite used to generate a `.PNG` and `.PDF` version of the Jira dashboard automatically. Also include a javascript function to enlarge member's profile pictures. And various other adjustments

## Warning !

You should consider running this in a container or VM as the chromium sandbox is disabled.
A Dockerfile and some scripts are provided to help setup with the setup.

## Installation
### Regular installation
- Install node and npm
- Install the pdfjam utility available with the tex suite
- Run: `$ nmp install` in the dashboard directory
- Copy the `settings_template.json` to `settings.json`
- Fill in [all the fields](#Settings) in the `settings.json` file

### Docker installation (recommended)
- Install [Docker](https://www.docker.com/), installation scripts are available for Ubuntu and CentOs in the `../scripts/` directory.
- Edit the `run.sh` file and change the `VOL_MNT` variable to the __absolute path__ where you want to map the `./out/` directory of the container. This is where the output files will be written by default.
- Copy the `settings_template.json` to `settings.json`
- Fill in [all the fields](#Settings) in the `settings.json` file

## Usage
### Regular usage
To generate output files run:

```$ node render.js```

To generate output files and send an email run:

```$ node renderAndMail.js```

To periodically generate files and send emails:

```$ node scheduledRender.js```

### Docker usage
To start `scheduledRender.js` as a daemon, use the `run.sh` script without any arguments:

```$ ./run.sh```

To execute `render.js` and `renderAndMail.js` once, append the desired script as an argument to the `run.sh` script:
```
$ ./run.sh render.js
$ ./run.sh renderAndMail.js   
```
When executing for the first time, docker will have to build the images, this can take several minutes.

## Settings

#### url <[string]>
URL of the target Jira dashboard ex: `https://your.domain.com/Dashboard.jspa`

#### worklogUrl <[string]>
The url of the worklog page to parse and scrape data from. Used for computing time averages of members

#### user <[string]>
Username of the authorized dashboard viewer

#### passwd <[string]>
Password of the authorized dashboard viewer

#### output <[string]>
Path of the `.PNG` destination file ex: `/home/user/dashboard.png`

#### outputPDF <[string]>
Path of the `.PDF` destination file ex: `/home/user/dashboard.pdf`

#### outputLandscapePDF <[string]>
Path of a `.PDF` destination file that will be printed in landscape format ex: `/home/user/dashboard_landscape.pdf`

#### startDate <[string]>
String representation of the start date of the project ex: `2018-06-14`

#### screenWidth <[number]>
Horizontal screen resolution of the simulated window to be opened by chromium.
Use this feature to adjust the layout and size of fonts in relations to horizontal width. To change the pixel density of the images produced, use the [deviceScalingFactor](#deviceScalingFactor) instead.

#### deviceScalingFactor <[number]>
Adjusts the subsampling factor. Effectively increases pixel density without affecting the layout. A `deviceScalingFactor` of `2` is recommended to generate printable results at `"screenWidth": 1920` with Letter (8.5" x 11") paper.

#### cron_schedule <[string]>
Schedules the update of the output documents. Use [cron syntax] to specify when the documents should be updated. For example, `0 * * * *` will update the files every hour at `XX:00`. Be mindful of the time required to render all documents.

#### cron_email <[string]>
Schedules the sending a new email containg the output files in attachement. Use [cron syntax] to specify when the documents should be sent. For example, `30 17 * * 4` will send an email every Thursday at 5:30 PM.

#### mail_transport <[Object]>
This [Object] contains the `options` required to connect to an SMTP server. These `options` get sent to the `createTransport(options[, defaults])` method of the Nodemailer library. Refer to the [documentation](https://nodemailer.com/smtp/) for the appropriate syntax. This is an example for a Gmail:
```json
"mail_transport": {
        "service": "gmail",
        "auth": {
            "user": "your.email@gmail.com",
            "pass": "gmailPassword123"
        }
    },
```
#### mail_from <[string]>
The *From* field of the automatic email. [Formatted addresses](https://nodemailer.com/message/addresses/) can be used to include name of sender. ex: `Ноде Майлер <foobar@example.com>`

#### mailing_list <[Array]<[string]>>
The list of email addresses to whom to send the email. ex:
```json
"mailing_list": [
    "foobar1@example.com",
    "foobar2@example.com",
    "foobar3@example.com",
]
```

[cron syntax]: http://www.nncron.ru/help/EN/working/cron-format.htm
[string]: https://developer.mozilla.org/en-US/docs/Web/JavaScript/Data_structures#String_type "String"
[number]: https://developer.mozilla.org/en-US/docs/Web/JavaScript/Data_structures#Number_type "Number"
[Object]: https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Object "Object"
[Array]: https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/Array "Array"
