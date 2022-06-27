#include <SPI.h>
#include "clock.h"

#define LED 13
#define DRDY 4
#define SYNC 10
// clock on pin 6

#define N_ADC 1
const int cs_pins[N_ADC] = {A0};

#define N_INACTIVE_ADC 0
const int inactive_cs_pins[N_INACTIVE_ADC] = {};

unsigned long sample_time = 0;

void setup() {
  pinMode(DRDY, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(SYNC, OUTPUT);

  digitalWrite(SYNC, HIGH);

  for (int i = 0; i < N_ADC; i++) {
    pinMode(cs_pins[i], OUTPUT);
    digitalWrite(cs_pins[i], HIGH);
  }

  for (int i = 0; i < N_INACTIVE_ADC; i++) {
    pinMode(inactive_cs_pins[i], OUTPUT);
    digitalWrite(inactive_cs_pins[i], HIGH);
  }

  delayMicroseconds(1);
  clockSetup();

  SPI.begin();
  
  SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE1));

  delay(1);

  while (Serial.read() != 'S') ;
  Serial.write(0xFA);
  digitalWrite(LED, HIGH);

  for (int i = 0; i < N_ADC; i++) {
    writeConfigRegisters(cs_pins[i]);
  }

  digitalWrite(SYNC, LOW);
  delayMicroseconds(10); // must be less than 2048*adc_clck
  digitalWrite(SYNC, HIGH);

  sample_time = micros();
}

void loop() {
  unsigned long t = micros();
  if(digitalRead(DRDY) == LOW) {
    // data is ready
    
    Serial.write("s"); // start of data
    
    unsigned long d = t - sample_time;
    sample_time = t;
    
    byte time_data[4] = {d>>24,(d>>16)%256,(d>>8)%256,d%256};
    Serial.write(time_data, 4);

    for (int i = 0; i < N_ADC; i++) {
      readData(cs_pins[i]);
      delayMicroseconds(1);
    }
  }
  delayMicroseconds(1);
}

void clear(byte *data, int len) {
  for (int i = 0; i < len; i++) {
    data[i] = 0;
  }
}

void transfer(int cs_pin, byte *data, int len) {
  digitalWrite(cs_pin, LOW);
  delayMicroseconds(1);
  SPI.transfer(data, len);
  delayMicroseconds(1);
  digitalWrite(cs_pin, HIGH);
}

// typical frame is 10 words, 3 bytes per word

void printRegisters(int cs_pin) {
  byte data[30]; // 1 whole frame to send command, response comes on next frame
  clear(data, 30);
  data[0] = 0b10100000;
  data[1] = 8; // read 9 registers
  transfer(cs_pin, data, 30);

  // data out: ack + 9 reg (+ crc, but we are ignoring)
  clear(data, 30);
  transfer(cs_pin, data, 30);

  Serial.println("registers:");
  for (int i = 0; i < 9; i++) {
    Serial.print(i, HEX);
    Serial.print(": ");
    Serial.print(data[3*i+3], HEX);
    Serial.print(" ");
    Serial.println(data[3*i+4], HEX);
  }
}

void writeConfigRegisters(int cs_pin) {
  byte data[30] = {
    0x61, 0x06, 0x00, // write command, reg 2-8
    0x05, 0x10, 0x00, // mode
    0xff, 0x16, 0x00, // clock (enable all channels, oversample 4096) - first byte 01 for single channel
    0x44, 0x44, 0x00, // gain1 (all channels gain 16)
    0x44, 0x44, 0x00, // gain2 (all channels gain 16)
    0x06, 0x00, 0x00, // cfg
    0x00, 0x00, 0x00, // threshold msb
    0x00, 0x00, 0x00, // threshold lsb
    0x00, 0x00, 0x00, // pad
  };
  transfer(cs_pin, data, 30);
}

void readData(int cs_pin) {
  byte data[30];
  clear(data, 30);
  transfer(cs_pin, data, 30);
  //response is: prev command response, 8 channels data, crc

  Serial.write(data+3, 8*3);
}
