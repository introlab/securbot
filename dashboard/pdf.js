const PDFDocument = require('pdfkit');
const fs = require('fs');
const imgSize = require('image-size');


module.exports.generatePDF = function (imagePath, pdfPath){
    var dim = imgSize(imagePath);
    // Adjusting pdf to 8 and 1/2 inches wide
    dim.height = dim.height / dim.width * 612;
    dim.width = 612;

    var doc = new PDFDocument({size: [dim.width, dim.height]});
    doc.pipe(fs.createWriteStream(pdfPath));
    doc.image(imagePath, 0, 0, {width: dim.width});
    doc.end();
}

module.exports.generateLandscapePDF = function (imagePath, pdfPath){
    var dim = imgSize(imagePath);

    // Computing number of pages necessary
    var vPixelPerPage = 8.5 / 11 * dim.width;
    var nPages = Math.ceil(dim.height / vPixelPerPage);

    // Opening landscape document
    var doc = new PDFDocument({
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
    doc.end();
}
