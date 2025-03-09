const express = require('express');
const http = require('http');
const socketIo = require("socket.io");
const mqtt = require('mqtt');

// MQTT Configuration
const MQTT_BROKER = "mqtt://test.mosquitto.org";
const MQTT_TOPIC_COMMAND = "robot/command";
const MQTT_TOPIC_STATUS = "robot/status";
const MQTT_TOPIC_WARNING = "robot/warning"; // New warning topic

// Initialize the app
const app = express();
const server = http.createServer(app);
const io = socketIo(server);
const PORT = 3000;

// MQTT Client Setup
const client = mqtt.connect(MQTT_BROKER);

client.on("connect", () => {
    console.log("Connected to MQTT broker");
    client.subscribe([MQTT_TOPIC_STATUS, MQTT_TOPIC_COMMAND, MQTT_TOPIC_WARNING], (err) => {
        if (err) {
            console.error("Failed to subscribe:", err);
        } else {
            console.log(`Subscribed to topics: ${MQTT_TOPIC_STATUS}, ${MQTT_TOPIC_COMMAND}, ${MQTT_TOPIC_WARNING}`);
        }
    });
});

client.on("message", (topic, message) => {
    try {
        const payload = JSON.parse(message.toString());

        if (topic === MQTT_TOPIC_STATUS) {
            // Extract values and emit to frontend
            const { value1 = 0, value2 = 0, value3 = 0 } = payload;
            io.emit("status", { value1, value2, value3 });
            console.log(`Status Updated: Value1=${value1}, Value2=${value2}, Value3=${value3}`);
        }

        if (topic === MQTT_TOPIC_WARNING) {
            // Send warning message to frontend
            io.emit("warning", payload);
            console.log(`Warning Received: ${payload}`);
        }

    } catch (error) {
        console.error("Error parsing MQTT message:", error);
    }
});

// Serve the HTML file
app.get("/", (req, res) => {
    res.sendFile(__dirname + "/index5.html");
});

// Socket.IO connection handler
io.on('connection', (socket) => {
    console.log('A user connected');

    socket.on('command', (data) => {
        console.log('Received command:', data);
        client.publish(MQTT_TOPIC_COMMAND, JSON.stringify(data));
    });

    socket.on('disconnect', () => {
        console.log('A user disconnected');
    });
});

server.listen(PORT, () => {
    console.log(`Server running at http://localhost:${PORT}`);
});
