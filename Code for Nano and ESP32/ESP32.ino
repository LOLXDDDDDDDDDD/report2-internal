#include "driver/adc.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/soc.h"
#include <PID_v1.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_bt.h>
#include <movingAvg.h>

const char *ssid = "SSID";
const char *password = "PASSWORD";

// Add your MQTT Broker IP address, example:
// const char* mqtt_server = "192.168.1.144";
const char *mqtt_server = "IP_ADDRESS";

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
bool button = false;

// Define Variables we'll be connecting to
double set = 0, pid_in = 0, pid_out = 0;

// Specify the links and initial tuning parameters
double Kp = 0.0012, Ki = 2.67, Kd = 0.014;
int base = 195, interval = 10; //base for base speed, interval for delay constant
PID pid(&pid_in, &pid_out, &set, Kp, Ki, Kd, DIRECT);

#define I2C_SLAVE_ADDR 0x08

// motor pin
#define r_blk 14
#define r_red 27
#define l_red 26
#define l_blk 25

movingAvg avg_l1(10);
movingAvg avg_l2(10);
movingAvg avg_l3(10);
movingAvg avg_r1(10);
movingAvg avg_r2(10);
movingAvg avg_r3(10);

movingAvg off_l1(300);
movingAvg off_l2(300);
movingAvg off_l3(300);
movingAvg off_r1(300);
movingAvg off_r2(300);
movingAvg off_r3(300);

unsigned long timer = 0, start_time = 0;
int i_l1 = 0, iAv_l1 = 0, iOff_l1 = 0;
int i_l2 = 0, iAv_l2 = 0, iOff_l2 = 0;
int i_l3 = 0, iAv_l3 = 0, iOff_l3 = 0;
int i_r1 = 0, iAv_r1 = 0, iOff_r1 = 0;
int i_r2 = 0, iAv_r2 = 0, iOff_r2 = 0;
int i_r3 = 0, iAv_r3 = 0, iOff_r3 = 0;

// pwm variable
const int freq = 5000;
const int pwm_r_red = 0;
const int pwm_r_blk = 1;
const int pwm_l_red = 2;
const int pwm_l_blk = 3;
const int resolution = 8;

// tmp sensor config
#ifdef __cplusplus
extern "C" {
#endif

uint8_t temprature_sens_read();

#ifdef __cplusplus
}
#endif

// wifi setup
void setup_wifi() {
    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

// message arrive via mqtt
void callback(char *topic, byte *message, unsigned int length) {
    Serial.print("Message arrived on topic: ");
    Serial.print(topic);
    Serial.print(". Message: ");
    String message_in;

    for (int i = 0; i < length; i++) {
        Serial.print((char)message[i]);
        message_in += (char)message[i];
    }
    Serial.println();

    // If a message is received on the topic esp32/output, you check if the message is either "on" or "off".
    // Changes the output state according to the message
    if (String(topic) == "esp32/Ray") {
        if (message_in == "on") {
            button = true;
        } else if (message_in == "off") {
            button = false;
        }
    }

    if (String(topic) == "esp32/Kp") {
        char buf[15];

        message_in.toCharArray(buf, 15); // string to char array

        Kp = atof(buf); // char array to double
        pid.SetTunings(Kp, Ki, Kd);
    }

    if (String(topic) == "esp32/Ki") {
        char buf[15];

        message_in.toCharArray(buf, 15); // string to char array

        Ki = atof(buf); // char array to double
        pid.SetTunings(Kp, Ki, Kd);
    }

    if (String(topic) == "esp32/Kd") {
        char buf[15];

        message_in.toCharArray(buf, 15); // string to char array

        Kd = atof(buf); // char array to double
        pid.SetTunings(Kp, Ki, Kd);
    }
}

// mqtt connect and retry if not
void reconnect() {
    // Loop until we're reconnected
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Attempt to connect
        if (client.connect("ESP8266Client")) {
            Serial.println("connected");
            // Subscribe
            client.subscribe("esp32/Ray");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

// drive the motor and brodcast via MQTT
void move(int l_pwm, int r_pwm) {

    char pid_c[10]; // weighted average as char[]
    dtostrf(pid_in, 10, 2, pid_c);
    // IR data as char
    char l1[10], l2[10], l3[10];
    char r1[10], r2[10], r3[10];

    sprintf(l1, "%i", iAv_l1);
    sprintf(l2, "%i", iAv_l2);
    sprintf(l3, "%i", iAv_l3);
    sprintf(r1, "%i", iAv_r1);
    sprintf(r2, "%i", iAv_r2);
    sprintf(r3, "%i", iAv_r3);

    char tmp[10]; // tmp of cpu
    sprintf(tmp, "%f", (temprature_sens_read() - 32) / 1.8);

    client.publish("esp32/l1", l1);
    client.publish("esp32/l2", l2);
    client.publish("esp32/l3", l3);
    client.publish("esp32/r1", r1);
    client.publish("esp32/r2", r2);
    client.publish("esp32/r3", r3);

    // button off
    if (!button) {
        ledcWrite(pwm_l_blk, 0);
        ledcWrite(pwm_l_red, 0);
        ledcWrite(pwm_r_blk, 0);
        ledcWrite(pwm_r_red, 0);
    }

    // button on
    else {

        ledcWrite(pwm_l_blk, 0);
        ledcWrite(pwm_l_red, 0);
        ledcWrite(pwm_r_blk, 0);
        ledcWrite(pwm_r_red, 0);
        delay(1);

        // if left not negative, as normal
        if (l_pwm > 0) {
            ledcWrite(pwm_l_blk, 0);
            ledcWrite(pwm_l_red, l_pwm);
        }

        // if negative, ground red
        else {
            ledcWrite(pwm_l_blk, l_pwm);
            ledcWrite(pwm_l_red, 0);
        }

        // same as above
        if (r_pwm > 0) {
            ledcWrite(pwm_r_blk, 0);
            ledcWrite(pwm_r_red, r_pwm);
        }

        else {
            ledcWrite(pwm_r_blk, r_pwm);
            ledcWrite(pwm_r_red, 0);
        }
    }
}

// read from nano
void IR_read() {
    bool status = false;
    Wire.requestFrom(I2C_SLAVE_ADDR, 12);
    byte dataget[12];

    for (int i = 0; i < 12; i++) {
        dataget[i] = Wire.read(); // Reading data higher byte first
        status = true;
    }

    if (!status) {
        Serial.println("I2C eorr");
    }

    else {
        // combing high byte and low byte
        i_l1 = dataget[0] << 8 | dataget[1];
        i_l2 = dataget[2] << 8 | dataget[3];
        i_l3 = dataget[4] << 8 | dataget[5];
        i_r1 = dataget[6] << 8 | dataget[7];
        i_r2 = dataget[8] << 8 | dataget[9];
        i_r3 = dataget[10] << 8 | dataget[11];
    }
}

// find offset (define straight line)
void calIRoffest() {
    unsigned long cal_time = millis();

    // execute porgrame for 5s
    while (millis() - cal_time < 5000) {

        IR_read();

        // ignore the result of first 500ms
        if (timer - cal_time < 500) {
            timer = millis();
            iOff_l2 = off_l1.reading(0);
            iOff_l1 = off_l2.reading(0);
            iOff_l3 = off_l3.reading(0);
            iOff_r1 = off_r1.reading(0);
            iOff_r2 = off_r2.reading(0);
            iOff_r3 = off_r3.reading(0);
        }

        else {
            iOff_l1 = off_l1.reading(i_l1);
            iOff_l2 = off_l2.reading(i_l2);
            iOff_l3 = off_l3.reading(i_l3);
            iOff_r1 = off_r1.reading(i_r1);
            iOff_r2 = off_r2.reading(i_r2);
            iOff_r3 = off_r3.reading(i_r3);
        }
    }
}

// get rms but no sqrt for more performance
void getIR() {
    iAv_l1 = avg_l1.reading(i_l1) - iOff_l1;
    iAv_l2 = avg_l2.reading(i_l2) - iOff_l2;
    iAv_l3 = avg_l3.reading(i_l3) - iOff_l3;
    iAv_r1 = avg_r1.reading(i_r1) - iOff_r1;
    iAv_r2 = avg_r2.reading(i_r2) - iOff_r2;
    iAv_r3 = avg_r3.reading(i_r3) - iOff_r3;
}

// calculate weighted average and process via pid
void IRcompute() {

    // length to center of bot
    double s_l1 = 0.65, s_l2 = 2, s_l3 = 3.2;
    double s_r1 = -0.65, s_r2 = -2, s_r3 = -3.2;

    // weighted average (left +ve, right -ve) (it is the input of pid)
    pid_in = (iAv_l1 * s_l1) + (iAv_l2 * s_l2) + (iAv_l3 * s_l3);
    pid_in += (iAv_r1 * s_r1) + (iAv_r2 * s_r2) + (iAv_r3 * s_r3);
    pid_in = pid_in / 6;

    // process weighted aveage using pid
    pid.Compute();
    // facing right positive
}

void setup() {
    Serial.begin(115200);

    // save power
    esp_bt_controller_disable();
    adc_power_off();

    // turn brownout off
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

    // start wifi
    setup_wifi();
    client.setServer(mqtt_server, 1883);

    Wire.begin();

    // turn the PID on
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(-255 + base, 255 - base);
    pid.SetSampleTime(interval);

    pinMode(r_blk, OUTPUT);
    pinMode(r_red, OUTPUT);
    pinMode(l_blk, OUTPUT);
    pinMode(l_red, OUTPUT);

    // setup pwm channel
    ledcSetup(pwm_l_blk, freq, resolution);
    ledcSetup(pwm_l_red, freq, resolution);
    ledcSetup(pwm_r_blk, freq, resolution);
    ledcSetup(pwm_r_red, freq, resolution);

    // attach pwm channel
    ledcAttachPin(l_blk, pwm_l_blk);
    ledcAttachPin(l_red, pwm_l_red);
    ledcAttachPin(r_blk, pwm_r_blk);
    ledcAttachPin(r_red, pwm_r_red);

    avg_l1.begin();
    avg_l2.begin();
    avg_l3.begin();
    avg_r1.begin();
    avg_r2.begin();
    avg_r3.begin();

    off_l1.begin();
    off_l2.begin();
    off_l3.begin();
    off_r1.begin();
    off_r2.begin();
    off_r3.begin();

    // define straghit line
    calIRoffest();

    client.setCallback(callback);
    start_time = millis();
}

void loop() {

    //read from nano
    IR_read();

    if (!client.connected()) {
        reconnect();
    }

    // update from pi
    client.loop();

    if (millis() - timer > (unsigned long)interval) {

        // ignore the result of first 500ms
        if (timer - start_time < 500) {
            timer = millis();
            iAv_l1 = avg_l1.reading(iOff_l1);
            iAv_l2 = avg_l2.reading(iOff_l2);
            iAv_l3 = avg_l3.reading(iOff_l3);
            iAv_r1 = avg_r1.reading(iOff_r1);
            iAv_r2 = avg_r2.reading(iOff_r2);
            iAv_r3 = avg_r3.reading(iOff_r3);
        }

        // after 500ms and button on
        else {

            // get rms but no sqrt
            getIR();

            // calculate weighted average and pid
            IRcompute();

            // if pid at max or min, self turn
            if (pid_out == 60) {
                move(base + pid_out, -(base + 46));
            }

            else if (pid_out == -60) {
                move(-(base + 46), base - pid_out);
            }

            //else work as normal
            else {
                move(base + pid_out, base - pid_out);
            }
        }
        timer = millis();
    }
}
