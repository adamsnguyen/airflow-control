const express = require('express'),
    app = express(),
    cors = require('cors'),
    morgan = require('morgan'),
    path = require('path'),
    bodyParser = require('body-parser'),
    mqtt = require('mqtt'),
    mqttClient = mqtt.connect('tcp://localhost:1883'),
    mqttAirTopic = 'airflowSpeed',
    mqttHeightTopic = 'objectHeight',
    mqttmotorSpeedTopic = 'motorSpeed',
    mqttSystem_property_JSON = 'system_property_JSON',
    mqttTopics = [mqttHeightTopic, mqttAirTopic, mqttmotorSpeedTopic, mqttSystem_property_JSON],
    server = require('http').createServer(app),
    io = require('socket.io').listen(server),
    debug = 1,
    debug_front = 1;

const kill = require('kill-port');

// serial port initialization:
const { SerialPort } = require('serialport')
const { ReadlineParser } = require('@serialport/parser-readline')
const port = new SerialPort({ path: '/dev/NANO', baudRate: 9800 })
const parser = port.pipe(new ReadlineParser({ delimiter: '\r\n' }))
const { ReadyParser } = require('@serialport/parser-ready')
parser.on('ready', () => console.log('the ready byte sequence has been received'))


parser.on('data', console.log)
//port.write('ROBOT PLEASE RESPOND\n')


port.on('open', openPort); // called when the serial port opens

function openPort() {
 
  function sendData() {
    console.log("worked");   
  }
}

// port.on('data', function (data) {
//     console.log('Data:', data);
//     const uint8 = new Uint8Array(1);
//     uint8[0]=1
//     port.write(uint8);
// });



var streamInterval;
var msFrequency = 20;

/* 
Subscribe (listen) to MQTT topic and start publishing
simulated data after successful MQTT connection 
*/
mqttClient.on('connect', () => {
    console.log('Mqtt connected.')

    for (let mqttTopic of mqttTopics) {
        mqttClient.subscribe(mqttTopic); //subscribe
        
    }
    // startStreamSimulation(); //publish
})

mqttClient.on('offline', () => {
    console.log('Mqtt offline.')
    for (let mqttTopic of mqttTopics) {
        mqttClient.unsubscribe(mqttTopic);
    }
    clearInterval(streamInterval);
})


// voltage{"data":"232.46","type":"voltage"}
// motorSpeed{"data":"233.93","type":"motorSpeed"}
// airflowSpeed{"data":"236.87","type":"airflowSpeed"}
// objectHeight{"data":"238.36","type":"objectHeight"}

/* 
Message event fires, when new messages
arrive on the subscribed topic
*/
mqttClient.on('message', function (topic, message) {
    /* console.log('Received: ' + message.toString() + ' from topic: ' + topic.toString()); */

    
    //if (topic === mqttHeightTopic) {
    let parsedMessage = JSON.parse(message);

    io.emit(topic, parsedMessage);
    if (debug) {
    //console.log(topic + message)
    }
    //}


})

/* 
Function that publishes simulated data to the MQTT broker every ≈20ms
*/
function startStreamSimulation() {

    var voltageData = 0,
        motorSpeedData = 0,
        airflowSpeedData = 0,
        objectHeightData = 0;


    streamInterval = setInterval(function () {

        /* Prepare random data */

        voltageData = returnRandomFloat(231, 233);
        motorSpeedData = returnRandomFloat(233, 235);
        airflowSpeedData = returnRandomFloat(235, 237);
        objectHeightData = returnRandomFloat(237, 239);

        // /* Publish random data to the corresponding MQTT topic as a JSON string  */
        // mqttClient.publish(mqttVoltageTopic, JSON.stringify({
        //     'data': voltageData,
        //     'type' : 'voltage'
        // }));

        // mqttClient.publish(mqttmotorSpeedTopic, JSON.stringify({
        //     'data': motorSpeedData,
        //     'type' : 'motorSpeed'
        // }));

        // mqttClient.publish(mqttAirTopic, JSON.stringify({
        //     'data': airflowSpeedData,
        //     'type' : 'airflowSpeed'
        // }));

        // // mqttClient.publish(mqttHeightTopic, JSON.stringify({
        // //     'data': objectHeightData,
        // //     'type' : 'objectHeight'
        // // }));


    }, msFrequency);
}

function returnRandomFloat(min, max) {
    return (Math.random() * (max - min) + min).toFixed(2);
}

io.on('connection', function(socket){
    socket.on('motor', function(data){
        mqttClient.publish("motor", data);
        console.log("Motor: " + data);
    });

    socket.on('offset', function(data){
        mqttClient.publish("offset", data);
        console.log("offset: " + data);
    });

    socket.on('rpmSetpoint', function(data){
        mqttClient.publish("rpmSetpoint", data);
        console.log("rpmSetpoint: " + data);
    });

    socket.on('p_term', function(data){
        mqttClient.publish("p_term", data);
        console.log("p_term: " + data);
    });

    socket.on('i_term', function(data){
        mqttClient.publish("i_term", data);
        console.log("i_term: " + data);
    });

    socket.on('d_term', function(data){
        mqttClient.publish("d_term", data);
        console.log("d_term: " + data);
    });

    socket.on('pid_toggle', function(data){
        mqttClient.publish("pid_toggle", data);
        //console.log("pid_toggle: " + data);
    });


    socket.on('height_setpoint', function(data){
        mqttClient.publish("height_setpoint", data);
        //console.log("height_setpoint: " + data);
    });

    socket.on('rpmMod', function(data){
        mqttClient.publish("rpmMod", data);
        //console.log("rpmMod: " + data);
    });

    socket.on('clear_error', function(data){
        mqttClient.publish("clear_error", data);
        //console.log("clear_error");
    });

    socket.on('base_rpm', function(data){
        mqttClient.publish("base_rpm", data);
        //console.log("clear_error");
    });

    socket.on('damperForward', function(data){
        console.log('Data:', data);
        const uint8 = new Uint8Array(1);
        uint8[0]=1
        port.write(uint8);
    });

    socket.on('damperBack', function(data){
        console.log('Data:', data);
        const uint8 = new Uint8Array(1);
        uint8[0]=2
        port.write(uint8);
    });

    socket.on('damperForwardCoarse', function(data){
        console.log('Data:', data);
        const uint8 = new Uint8Array(1);
        uint8[0]=3
        port.write(uint8);
    });

    socket.on('damperBackCoarse', function(data){
        console.log('Data:', data);
        const uint8 = new Uint8Array(1);
        uint8[0]=4
        port.write(uint8);
    });

 });


app.use(bodyParser.json()); // to support JSON-encoded bodies
app.use(bodyParser.urlencoded({ // to support URL-encoded bodies
    extended: true
}));

app.get('/', function (req, res) {
    res.send(
        [{
            title: "Hi, I'm the express server!",
            description: "Start Moquette and the client application to see the action!"
        }]
    )
});


server.listen(3000 , '0.0.0.0', function () {
    console.log('App listening on port 3000!');
});


