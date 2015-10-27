var socket = io.connect('/');
// socket.emit('load finish');
$(document).ready(function() {
  $('#start-button').click(function(){
    socket.emit('control message','START');
  });
  $('#stop-button').click(function(){
    socket.emit('control message','STOP');
  });
});
