/*
 WiFi Web Server LED Blink

 A simple web server that lets you blink an LED via the web.
 This sketch will print the IP address of your WiFi Shield (once connected)
 to the Serial monitor. From there, you can open that address in a web browser
 to turn on and off the LED on pin 5.

 If the IP address of your shield is yourAddress:
 http://yourAddress/H turns the LED on
 http://yourAddress/L turns it off

 This example is written for a network using WPA2 encryption. For insecure
 WEP or WPA, change the Wifi.begin() call and use Wifi.setMinSecurity() accordingly.

 Circuit:
 * WiFi shield attached
 * LED attached to pin 5

 created for arduino 25 Nov 2012
 by Tom Igoe

ported for sparkfun esp32 
31.01.2017 by Jan Hendrik Berlin
 
 */

#include <WiFi.h>

#define LED_BUILTIN 2
#define BUF_SZ 1024

const char* ssid = "TP-Link_9457";
const char* password = "17318860";

WiFiServer server(80);
bool server_started = false;

void start_server() {
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);


    const IPAddress ip(192, 168, 0, 120);
    const IPAddress subnet(255, 255, 255, 0);

    if (!WiFi.config(ip, ip, subnet)) {
        Serial.println("Failed to configure!");
    }

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected.");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    server.begin();
    Serial.println("server.begin");

    server_started = true;
}

void setup() {
    Serial.println("setup");

    Serial.begin(115200);
    Serial2.begin(115200);
    pinMode(LED_BUILTIN, OUTPUT);  // set the LED pin mode

    delay(10);

    // We start by connecting to a WiFi network

    start_server();

    motor_setup();
}

WiFiClient client;

void loop() {
    if (WiFi.status() != WL_CONNECTED) {

        Serial.println("Reconnecting to WiFi...");
        WiFi.disconnect();
        WiFi.reconnect();

        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
        Serial.println("");

        return;
    }

    if(! client){

        client = server.available();  // listen for incoming clients
        if (client){

            Serial.println("New Client.");
        }
    }

    if (client) {
        String currentLine = "";
        uint8_t buf1[BUF_SZ];
        uint8_t cmd[256];
        int idx = 0;
        int buf_len = 0;

        if(client.connected()) {     // loop while the client's connected
            if (client.available()) {    // if there's bytes to read from the client,
                char c = client.read();  // read a byte, then
                Serial.write(c);         // print it out the serial monitor

                if (idx < 256) {
                    cmd[idx] = c;
                    idx++;
                }

                if (c == '\n') {

                    // char cbuf[256];
                    // sprintf(cbuf, "%d", buf_len);
                    // client.print(cbuf);
                    // client.write(buf1, buf_len);
                    buf_len = 0;
                    idx = 0;
                    currentLine = "";

                    digitalWrite(LED_BUILTIN, LOW);
                } else {
                    currentLine += c;

                    digitalWrite(LED_BUILTIN, HIGH);
                }
            }

            if (Serial2.available()) {
                uint8_t buf2[BUF_SZ];

                size_t sz = Serial2.readBytes(buf2, BUF_SZ);
                // if (buf_len + sz < BUF_SZ) {
                //     memcpy(buf1 + buf_len, buf2, sz);
                //     buf_len += sz;
                // }
                client.write(buf2, sz);
            }
        }

        // delay(3000);
        // // close the connection:
        // client.stop();
        // Serial.println("Client Disconnected.");
    }

    motor_loop();
}
