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

struct EncoderData {
    char  mark[4];
    int   msec;    
    int   counts[2];
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
        uint8_t cmd[6];
        int idx = 0;

        if(client.connected()) {     // loop while the client's connected
            while(client.available()) {    // if there's bytes to read from the client,
                char c = client.read();  // read a byte, then

                switch(idx){
                case 0:
                    if(c == '\xFF'){
                        idx++
                        ;
                        digitalWrite(LED_BUILTIN, HIGH);
                    }
                    break;                    
                case 1:
                    if(c == '\xEE'){
                        idx++;
                    }
                    else{
                        idx = 0;
                    }
                    break;

                default:
                    cmd[idx] = c;

                    idx++;
                    if(idx == 2 + 2 + 2){
                        short pwm_l = *(short*)(cmd + 2);
                        short pwm_r = *(short*)(cmd + 4);

                        Serial.printf("PWM %d %d\n", pwm_l, pwm_r);

                        if(-255 <= pwm_l && pwm_l <= 255 && -255 <= pwm_r && pwm_r <= 255){
                            motor_loop(pwm_l, pwm_r);
                        }

                        idx = 0;
                        digitalWrite(LED_BUILTIN, LOW);
                    }
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

                    Serial.printf("IMU:%d acc:(%.1f, %.1f, %.1f) gyro:(%.1f, %.1f, %.1f) temp:%.1f\n", sizeof(IMUdata), imu_dt.acc_x, imu_dt.acc_y, imu_dt.acc_z, imu_dt.gyro_x, imu_dt.gyro_y, imu_dt.gyro_z, imu_dt.temp);

                    EncoderData enc_dt;

                    enc_dt.mark[0] = 0xCC;
                    enc_dt.mark[1] = 0x77;
                    enc_dt.mark[2] = 0xCC;
                    enc_dt.mark[3] = 0x77;

                    get_encoder_counts(enc_dt.msec, enc_dt.counts[0], enc_dt.counts[1]);

                    Serial.printf("enc %d msec %d %d\n", enc_dt.msec, enc_dt.counts[0], enc_dt.counts[1]);

                    client.write(buf2, gap);
                    client.write((const char*)&imu_dt, sizeof(imu_dt));
                    client.write((const char*)&enc_dt, sizeof(enc_dt));
                    client.write(buf2 + gap, sz - gap);
                }
            }
        }

        // delay(3000);
        // // close the connection:
        // client.stop();
        // Serial.println("Client Disconnected.");
    }
}
