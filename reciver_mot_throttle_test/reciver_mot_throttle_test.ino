volatile uint32_t current_time;
volatile uint32_t last_channel_1 = 0;
volatile uint32_t last_channel_2 = 0;
volatile uint32_t last_channel_3 = 0;
volatile uint32_t last_channel_4 = 0;
volatile uint32_t last_channel_5 = 0;
volatile uint32_t last_channel_6 = 0;
volatile uint32_t timer_1;
volatile uint32_t timer_2;
volatile uint32_t timer_3;
volatile uint32_t timer_4;
volatile uint32_t timer_5;
volatile uint32_t timer_6;

volatile uint32_t ReceiverValue[5];

const int channel_1_pin = 34;
const int channel_2_pin = 35;
const int channel_3_pin = 32;
const int channel_4_pin = 33;
const int channel_5_pin = 25;  // Change the pin number accordingly
const int channel_6_pin = 26;  // Change the pin number accordingly


#include <ESP32Servo.h>
Servo mot1;
float throttle;
const int mot1_esc=18;

void channelInterruptHandler() {
  current_time = micros();

  // Channel 1
  if (digitalRead(channel_1_pin)) {
    if (last_channel_1 == 0) {
      last_channel_1 = 1;
      timer_1 = current_time;
    }
  } else if (last_channel_1 == 1) {
    last_channel_1 = 0;
    ReceiverValue[0] = current_time - timer_1;
  }

  // Channel 2
  if (digitalRead(channel_2_pin)) {
    if (last_channel_2 == 0) {
      last_channel_2 = 1;
      timer_2 = current_time;
    }
  } else if (last_channel_2 == 1) {
    last_channel_2 = 0;
    ReceiverValue[1] = current_time - timer_2;
  }

  // Channel 3
  if (digitalRead(channel_3_pin)) {
    if (last_channel_3 == 0) {
      last_channel_3 = 1;
      timer_3 = current_time;
    }
  } else if (last_channel_3 == 1) {
    last_channel_3 = 0;
    ReceiverValue[2] = current_time - timer_3;
  }

  // Channel 4
  if (digitalRead(channel_4_pin)) {
    if (last_channel_4 == 0) {
      last_channel_4 = 1;
      timer_4 = current_time;
    }
  } else if (last_channel_4 == 1) {
    last_channel_4 = 0;
    ReceiverValue[3] = current_time - timer_4;
  }

  // Channel 5
if (digitalRead(channel_5_pin)) {
  if (last_channel_5 == 0) {
    last_channel_5 = 1;
    timer_5 = current_time;
  }
} else if (last_channel_5 == 1) {
  last_channel_5 = 0;
  ReceiverValue[4] = current_time - timer_5;
}

// Channel 6
if (digitalRead(channel_6_pin)) {
  if (last_channel_6 == 0) {
    last_channel_6 = 1;
    timer_6 = current_time;
  }
} else if (last_channel_6 == 1) {
  last_channel_6 = 0;
  ReceiverValue[5] = current_time - timer_6;
}

}

void setup() {
  Serial.begin(115200);

  pinMode(channel_1_pin, INPUT_PULLUP);
  pinMode(channel_2_pin, INPUT_PULLUP);
  pinMode(channel_3_pin, INPUT_PULLUP);
  pinMode(channel_4_pin, INPUT_PULLUP);
  pinMode(channel_5_pin, INPUT_PULLUP);
  pinMode(channel_6_pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(channel_1_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_2_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_3_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_4_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_5_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_6_pin), channelInterruptHandler, CHANGE);

 mot1.attach(mot1_esc,1000,2000);
  while (ReceiverValue[2] < 1020 ||
    ReceiverValue[2] > 1050) {
      delay(4);
    }
}

void loop() {
  // Your main code here
  // Use ReceiverValue array as needed

   delayMicroseconds(3500);
  Serial.print(ReceiverValue[0]);
  Serial.print(" /t ");
  Serial.print(ReceiverValue[1]);
  Serial.print(" /t ");
  Serial.print(ReceiverValue[2]);
  Serial.print(" /t ");
  Serial.print(ReceiverValue[3]);
  Serial.print(" /t ");
  Serial.print(ReceiverValue[4]);
  Serial.print(" /t ");
  Serial.print(ReceiverValue[5]);
  Serial.print(" /t ");
  Serial.print(throttle);
  Serial.println(" ");

  
  throttle=map(ReceiverValue[2],1000,2000,0,180);


  mot1.write(throttle);

}


