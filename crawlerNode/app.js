
var SerialPort = require("serialport");
var express = require('express');
var app = express();
var http = require('http').Server(app);
var io = require('socket.io')(http);
var path = require('path');


app.set('views', path.join(__dirname, 'views'));
app.set('view engine', 'ejs');

var routes = require('./routes/index');
app.use('/', routes);
app.use(express.static(path.join(__dirname, 'public')));



var portName = process.argv[2],
portConfig = {
	baudRate: 9600,
	parser: SerialPort.parsers.readline("\n")
};

var sp;

sp = new SerialPort.SerialPort(portName, portConfig);

io.on('connection', function(socket){
  console.log('a user connected');
  socket.on('disconnect', function(){
  });

  socket.on('control message', function(msg){
    // console.log(msg);
    sp.write(msg + "\n");
  });
});

http.listen(3000, function(){
  console.log('listening on *:3000');
});

sp.on("open", function () {

  // Clear any remaining streams from previous.
  sp.flush();

  console.log('open');
  sp.on('data', function(data) {
    console.log('Output: ' + data);
    // io.emit("ack message", data);
  });
});

module.exports = app;
