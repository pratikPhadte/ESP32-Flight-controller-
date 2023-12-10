volatile uint32_t current_time;
volatile uint32_t last_channel_1 = 0;
volatile uint32_t last_channel_2 = 0;
volatile uint32_t last_channel_3 = 0;
volatile uint32_t last_channel_4 = 0;
volatile uint32_t timer_1;
volatile uint32_t timer_2;
volatile uint32_t timer_3;
volatile uint32_t timer_4;
volatile uint32_t receiver_input[5];

const int channel_1_pin = 34;
const int channel_2_pin = 26;
const int channel_3_pin = 27;
const int channel_4_pin = 14;

float throttle;
const int mot_esc=18;

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
    receiver_input[1] = current_time - timer_1;
  }

  // Channel 2
  if (digitalRead(channel_2_pin)) {
    if (last_channel_2 == 0) {
      last_channel_2 = 1;
      timer_2 = current_time;
    }
  } else if (last_channel_2 == 1) {
    last_channel_2 = 0;
    receiver_input[2] = current_time - timer_2;
  }

  // Channel 3
  if (digitalRead(channel_3_pin)) {
    if (last_channel_3 == 0) {
      last_channel_3 = 1;
      timer_3 = current_time;
    }
  } else if (last_channel_3 == 1) {
    last_channel_3 = 0;
    receiver_input[3] = current_time - timer_3;
  }

  // Channel 4
  if (digitalRead(channel_4_pin)) {
    if (last_channel_4 == 0) {
      last_channel_4 = 1;
      timer_4 = current_time;
    }
  } else if (last_channel_4 == 1) {
    last_channel_4 = 0;
    receiver_input[4] = current_time - timer_4;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(channel_1_pin, INPUT_PULLUP);
  pinMode(channel_2_pin, INPUT_PULLUP);
  pinMode(channel_3_pin, INPUT_PULLUP);
  pinMode(channel_4_pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(channel_1_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_2_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_3_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_4_pin), channelInterruptHandler, CHANGE);
  
  pinMode(mot_esc1,OUTPUT);
  //analogWriteFrequency(1, 250);
  //analogWriteResolution(12);
  delay(250);
  while (reciever_input[1] < 1020 ||
    reciever_input[1] > 1050) {
      delay(4);
    }
}

void loop() {
  // Your main code here
  // Use receiver_input array as needed

   delayMicroseconds(3500);
  Serial.print(receiver_input[1]);
  Serial.print(" /t ");
  Serial.print(receiver_input[2]);
  Serial.print(" /t ");
  Serial.print(receiver_input[3]);
  Serial.print(" /t ");
  Serial.print(receiver_input[4]);
  Serial.print(" /t ");
  Serial.println(" ");

  throttle=ReceiverValue[1];
  analogWrite(mot_esc1,1.024*throttle);
}
