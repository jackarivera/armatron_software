const express = require('express');
const app = express();
const PORT = 4000;

// For demonstration, a simple route:
app.get('/health', (req, res) => {
  res.json({ status: 'ok', message: 'Server up and running' });
});

// TODO: Add endpoints that talk to your C++ driver or microservice
// e.g., GET /can/motor/:id/on => calls into the controls app 
// (We'll handle bridging in a future step.)

app.listen(PORT, () => {
  console.log(`Server listening on port ${PORT}`);
});
