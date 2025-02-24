const express = require('express');
const http    = require('http');
const socketIO= require('socket.io');
const path    = require('path');
const net     = require('net');
const app     = express();
const server  = http.createServer(app);
const io      = socketIO(server);

const SOCKET_PATH = '/tmp/robot_socket';

let client = null; // we'll hold a single connection to the daemon

function connectToDaemon() {
  client = net.createConnection(SOCKET_PATH, ()=>{
    console.log("[Node] Connected to real_time_daemon via UNIX socket.");
  });
  client.on('error', (err)=>{
    console.error("[Node] Socket error: ", err);
    setTimeout(connectToDaemon, 2000);
  });
  client.on('close', ()=>{
    console.log("[Node] Socket closed, retry in 2s...");
    setTimeout(connectToDaemon, 2000);
  });
  let buffer = '';
  client.on('data', (data)=>{
    buffer += data.toString();
    let lines = buffer.split('\n');
    buffer = lines.pop(); // last is partial
    lines.forEach(line=>{
      if(line.trim().length>0) {
        // parse JSON
        try {
          let msg = JSON.parse(line);
          // e.g. { "type":"motorStates", "motors":{"1":{...}}}
          if(msg.type === 'motorStates'){
            // broadcast to all websockets
            io.emit('motorStates', msg);
          }
        } catch(e){
          console.error("[Node] parse error: ", e);
        }
      }
    });
  });
}

// Express static
app.use(express.static(path.join(__dirname, 'public')));

// Example route
app.get('/hello', (req,res)=> {
  res.send("Hello from Node web_app!");
});

// Socket.IO events from the browser
io.on('connection', (socket)=>{
  console.log("[Node] Browser connected via socket.io");

  // Browser can send commands like: {cmd:'motorOn', motorID:1}
  socket.on('sendCommand', (msg)=>{
    // forward to daemon
    if(client){
      let line = JSON.stringify(msg) + "\n";
      client.write(line);
    }
  });
});

// Start server
server.listen(8888, ()=>{
  console.log("[Node] Web server on http://<BBB_IP>:8080");
});

// Connect to the daemon
connectToDaemon();
