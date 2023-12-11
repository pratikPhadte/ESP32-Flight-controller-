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
volatile int ReceiverValue[6]; // Increase the array size to 6 for Channel 1 to Channel 6
const int channel_1_pin = 34;
const int channel_2_pin = 35;
const int channel_3_pin = 32;
const int channel_4_pin = 33;
const int channel_5_pin = 25;
const int channel_6_pin = 26;
 
 void channelInterruptHandler()
{
  current_time = micros();
  // Channel 1
  if (digitalRead(channel_1_pin))
  {
    if (last_channel_1 == 0)
    {
      last_channel_1 = 1;
      timer_1 = current_time;
    }
  }
  else if (last_channel_1 == 1)
  {
    last_channel_1 = 0;
    ReceiverValue[0] = current_time - timer_1;
  }

  // Channel 2
  if (digitalRead(channel_2_pin))
  {
    if (last_channel_2 == 0)
    {
      last_channel_2 = 1;
      timer_2 = current_time;
    }
  }
  else if (last_channel_2 == 1)
  {
    last_channel_2 = 0;
    ReceiverValue[1] = current_time - timer_2;
  }

  // Channel 3
  if (digitalRead(channel_3_pin))
  {
    if (last_channel_3 == 0)
    {
      last_channel_3 = 1;
      timer_3 = current_time;
    }
  }
  else if (last_channel_3 == 1)
  {
    last_channel_3 = 0;
    ReceiverValue[2] = current_time - timer_3;
  }

  // Channel 4
  if (digitalRead(channel_4_pin))
  {
    if (last_channel_4 == 0)
    {
      last_channel_4 = 1;
      timer_4 = current_time;
    }
  }
  else if (last_channel_4 == 1)
  {
    last_channel_4 = 0;
    ReceiverValue[3] = current_time - timer_4;
  }

  // Channel 5
  if (digitalRead(channel_5_pin))
  {
    if (last_channel_5 == 0)
    {
      last_channel_5 = 1;
      timer_5 = current_time;
    }
  }
  else if (last_channel_5 == 1)
  {
    last_channel_5 = 0;
    ReceiverValue[4] = current_time - timer_5;
  }

  // Channel 6
  if (digitalRead(channel_6_pin))
  {
    if (last_channel_6 == 0)
    {
      last_channel_6 = 1;
      timer_6 = current_time;
    }
  }
  else if (last_channel_6 == 1)
  {
    last_channel_6 = 0;
    ReceiverValue[5] = current_time - timer_6;
  }
}


void neutralPositionAdjustment()
{
  int min = 1490;
  int max = 1510;
  if (ReceiverValue[0] < max && ReceiverValue[0] > min)
  {
    ReceiverValue[0]= 1500;
  } 
  if (ReceiverValue[1] < max && ReceiverValue[1] > min)
  {
    ReceiverValue[1]= 1500;
  } 
  if (ReceiverValue[3] < max && ReceiverValue[3] > min)
  {
    ReceiverValue[3]= 1500;
  } 
  if(ReceiverValue[0]==ReceiverValue[1] && ReceiverValue[1]==ReceiverValue[3] && ReceiverValue[3]==ReceiverValue[0] )
  {
    ReceiverValue[0]= 1500;
    ReceiverValue[1]= 1500;
    ReceiverValue[3]= 1500;
  }
}

void setup() {
  Serial.begin(115200);

int led_time=100;
 pinMode(15, OUTPUT);
  digitalWrite(15, LOW);
  delay(led_time);
  digitalWrite(15, HIGH);
  delay(led_time);
  digitalWrite(15, LOW);
  delay(led_time);
  digitalWrite(15, HIGH);
  delay(led_time);
  digitalWrite(15, LOW);
  delay(led_time);
  digitalWrite(15, HIGH);
  delay(led_time);
  digitalWrite(15, LOW);
  delay(led_time);
  digitalWrite(15, HIGH);
  delay(led_time);
  digitalWrite(15, LOW);
  delay(led_time);


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
  
}

void loop() {
   channelInterruptHandler();
  neutralPositionAdjustment();
//Reciever signals
  Serial.print(ReceiverValue[0]);
  Serial.print(" - ");
  Serial.print(ReceiverValue[1]);
  Serial.print(" - ");
  Serial.print(ReceiverValue[2]);
  Serial.print(" - ");
  Serial.print(ReceiverValue[3]);
  Serial.print(" - ");
  Serial.print(ReceiverValue[4]);
  Serial.print(" - ");
  Serial.print(ReceiverValue[5]);
  Serial.println(" -- ");
}
