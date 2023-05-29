#! /usr/bin/env node

const express = require('express');
const app = express();

apppath = __dirname + "/app/"


// Standard express website boot
app.use(express.static(apppath));

app.get('/', async (request, response) => {
    response.send( await readFile('index.html', 'utf8') );
});

app.listen(process.env.PORT || 3000, () => {
    console.log(`NODE: App available on http://127.0.0.1:3000`)
});