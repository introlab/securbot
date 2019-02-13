const nodemailer = require("nodemailer");

module.exports.send = async function(settings){

    console.log('Sending dashboard throught email');

    var transporter = nodemailer.createTransport(settings.mail_transport);

    var mailOptions = {
        from: settings.mail_from,
        to: settings.mailing_list.join(', '),
        subject: 'Securbot: tableu de bord hebdomadaire',
        html: 'Vous trouverez ci-joint le tableau de bord de notre équipe',
        attachments: [
            {path: settings.output},
            {path: settings.outputPDF}
        ]
    }

    return transporter.sendMail(mailOptions);
}
