#include <Wire.h> // include Wire library

#define l1 0 // A0
#define l2 3 // A3
#define l3 7 // A7
#define r1 1 // A1
#define r2 2 // A2
#define r3 6 // A6

#define I2C_SLAVE_ADDR 0x08 // slave address

int i_l1 = 0;
int i_l2 = 0;
int i_l3 = 0;
int i_r1 = 0;
int i_r2 = 0;
int i_r3 = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin(I2C_SLAVE_ADDR); // join i2c bus with address 8
    pinMode(l1, INPUT);
    pinMode(l2, INPUT);
    pinMode(l3, INPUT);
    pinMode(r1, INPUT);
    pinMode(r2, INPUT);
    pinMode(r3, INPUT);
    Wire.onRequest(requestEvent); // create a receive event
}
void loop() {
    i_l1 = analogRead(l1);
    i_l2 = analogRead(l2);
    i_l3 = analogRead(l3);
    i_r1 = analogRead(r1);
    i_r2 = analogRead(r2);
    i_r3 = analogRead(r3);
    delay(1);
}
void requestEvent() {
    //split int into higher and lower byte 
    //as value is greater than 255
    Wire.write(highByte(i_l1));
    Wire.write(lowByte(i_l1));

    Wire.write(highByte(i_l2));
    Wire.write(lowByte(i_l2));

    Wire.write(highByte(i_l3));
    Wire.write(lowByte(i_l3));

    Wire.write(highByte(i_r1));
    Wire.write(lowByte(i_r1));

    Wire.write(highByte(i_r2));
    Wire.write(lowByte(i_r2));

    Wire.write(highByte(i_r3));
    Wire.write(lowByte(i_r3));
}