const pdfDocument = require('pdfkit');
const fs = require('fs');
const imgSize = require('image-size');
const pdfjam = require('pdfjam');
const retry = require('async-retry');

module.exports.generatePDF = function (imagePath, pdfPath){
    console.log('Generating single page')
    var dim = imgSize(imagePath);
    // Adjusting pdf to 8 and 1/2 inches wide
    dim.height = dim.height / dim.width * 612;
    dim.width = 612;

    var doc = new pdfDocument({size: [dim.width, dim.height]});
    doc.pipe(fs.createWriteStream(pdfPath));
    doc.image(imagePath, 0, 0, {width: dim.width});
    doc.end();
}

module.exports.generateLandscapePDF = async function (imagePath, pdfPath,
    margins = {hMar : 0, vMar : 20})
{
    console.log('Generating landscape PDF');
    var dim = imgSize(imagePath);

    // Computing number of pages necessary
    var vPixelPerPage = 8.5 / 11 * dim.width;
    var nPages = Math.ceil(dim.height / vPixelPerPage);

    // Opening landscape document
    var doc = new pdfDocument({
        size: 'letter',
        layout: 'landscape'
    });
    doc.pipe(fs.createWriteStream(pdfPath));

    // Printing images
    doc.image(imagePath, 0, 0, {width: 792.0});
    for (var i = 1; i < nPages; ++i)
    {
        doc.addPage();
        doc.image(imagePath, 0, -612*i, {width: 792.0});
    }
    await doc.end();

    // Adding margins to pages
    console.log('Adding margins')
    await retry( async (abort, attempt) => {
        await pdfjam(pdfPath, {
            outfile: pdfPath,
            orientation: 'landscape',
            papersize: '{8.5in,11in}',
            trim: `-${margins.hMar}pts -${margins.vMar}pts -${margins.hMar}pts -${margins.vMar}pts`
        });
    }, {minTimeout: 10});
}
