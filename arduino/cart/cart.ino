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
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define LED_BUILTIN 2
#define BUF_SZ 1024

struct IMUdata {
    char  mark[4];    
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temp;
};

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
    imu_setup();
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

                int gap = -1;
                for(int i = 0; i < sz - 3; i++){
                    if(buf2[i] == 0xAA && buf2[i + 1] == 0x55 && buf2[i + 2] == 1){
                        gap = i;
                        break;
                    }
                }
                if(gap == -1){

                    client.write(buf2, sz);
                }
                else{
                    sensors_event_t acc, gyro, temp;
                    IMUdata imu_dt;

                    imu_loop(acc, gyro, temp);

                    imu_dt.mark[0] = 0xBB;
                    imu_dt.mark[1] = 0x66;
                    imu_dt.mark[2] = 0xBB;
                    imu_dt.mark[3] = 0x66;

                    imu_dt.acc_x = acc.acceleration.x;
                    imu_dt.acc_y = acc.acceleration.y;
                    imu_dt.acc_z = acc.acceleration.z;

                    imu_dt.gyro_x = gyro.gyro.x;
                    imu_dt.gyro_y = gyro.gyro.y;
                    imu_dt.gyro_z = gyro.gyro.z;

                    imu_dt.temp   = temp.temperature;

                    client.write(buf2, gap);

                    client.write((const char*)&imu_dt, sizeof(imu_dt));

                    client.write(buf2 + gap, sz - gap);

                    Serial.printf("IMU:%d acc:(%.1f, %.1f, %.1f) gyro:(%.1f, %.1f, %.1f) temp:%.1f\n", sizeof(IMUdata), imu_dt.acc_x, imu_dt.acc_y, imu_dt.acc_z, imu_dt.gyro_x, imu_dt.gyro_y, imu_dt.gyro_z, imu_dt.temp);
                }
            }
        }

        // delay(3000);
        // // close the connection:
        // client.stop();
        // Serial.println("Client Disconnected.");
    }

    motor_loop();


}
