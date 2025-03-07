const express = require('express');
const http    = require('http');
const socketIO= require('socket.io');
const path    = require('path');
const net     = require('net');
const app     = express();
const server  = http.createServer(app);
const io      = socketIO(server);

const SOCKET_PATH = '/tmp/robot_socket';
const DEBUG = true;

let client = null; // we'll hold a single connection to the daemon

function connectToDaemon() {
  client = net.createConnection(SOCKET_PATH, () => {
    console.log("[Node] Connected to real_time_daemon via UNIX socket.");
  });
  client.on('error', (err) => {
    console.error("[Node] Socket error:", err);
    setTimeout(connectToDaemon, 2000);
  });
  client.on('close', () => {
    console.log("[Node] Socket closed, retrying in 2s...");
    setTimeout(connectToDaemon, 2000);
  });
  let buffer = '';
  client.on('data', (data) => {
    buffer += data.toString();
    let lines = buffer.split('\n');
    buffer = lines.pop(); // the last part may be partial
    lines.forEach((line) => {
      line = line.trim();
      if (line.length === 0) return;
      // Basic validation: check that the line starts with '{' and ends with '}'
      if (line[0] !== '{' || line[line.length - 1] !== '}') {
        console.error("[Node] Received invalid JSON line, skipping:", line);
        return;
      }
      try {
        let msg = JSON.parse(line);
        console.log(`[Node] Received JSON: ${line}`); // Debug print
        if (msg.type === 'motorStates') {
          io.emit('motorStates', msg);
        }
      } catch (e) {
        console.error("[Node] parse error: ", e);
      }
    });
    // Add an immediate read event to ensure fast reading
    process.nextTick(() => {
      client.resume();
  });
  });
}

// Serve static files from the 'public' folder
app.use(express.static(path.join(__dirname, 'public')));

// Example route
app.get('/hello', (req, res) => {
  res.send("Hello from Node web_app!");
});

// Socket.IO events from the browser
io.on('connection', (socket) => {
  console.log("[Node] Browser connected via socket.io");

  socket.on('sendCommand', (msg) => {
    if (DEBUG) {
      console.log("[Node] Received command from browser:", msg);
    }
    // Forward command to daemon
    if(client) {
      let line = JSON.stringify(msg) + "\n";
      client.write(line);
      if (DEBUG) {
        console.log("[Node] Command forwarded to daemon:", line);
      }
    } else {
      console.error("[Node] No client connection available to forward command.");
    }
  });
});

server.listen(8888, () => {
  console.log("[Node] Web server running on http://<BBB_IP>:8888");
});

// Connect to the daemon
connectToDaemon();
