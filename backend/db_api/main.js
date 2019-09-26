const MongoClient = require('mongodb').MongoClient;


// MongoDB environment variables
const DB_HOST = process.env.DB_HOST //|| 'localhost';
const DB_PORT = process.env.DB_PORT //|| '27017';
const DB_URI = `mongodb://${DB_HOST}:${DB_PORT}/securbot`;

MongoClient.connect(DB_URI, (err, db) => {
    if (err) throw err;
    console.log('Success');
    db.close()
});
