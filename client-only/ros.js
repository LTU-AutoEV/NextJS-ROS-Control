var roslib = require('roslib')

var ros = new roslib.Ros({
  url : 'ws://localhost:9090/'
})

ros.on('connection', function() {
  console.log('roslib: Connected to websocket server.');
})

ros.on('error', function(error) {
  console.log('roslib: Error connecting to websocket server: ', error);
})

ros.on('close', function() {
  console.log('roslib: Connection to websocket server closed.');
})

module.exports = ros
