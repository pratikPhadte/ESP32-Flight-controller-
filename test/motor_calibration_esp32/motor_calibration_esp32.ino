
//THERE IS NO WARRANTY FOR THE SOFTWARE, TO THE EXTENT PERMITTED BY APPLICABLE LAW. EXCEPT WHEN OTHERWISE STATED IN WRITING THE COPYRIGHT HOLDERS AND/OR 
//OTHER PARTIES PROVIDE THE SOFTWARE “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
//OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE SOFTWARE IS WITH THE CUSTOMER. SHOULD THE 
//SOFTWARE PROVE DEFECTIVE, THE CUSTOMER ASSUMES THE COST OF ALL NECESSARY SERVICING, REPAIR, OR CORRECTION EXCEPT TO THE EXTENT SET OUT UNDER THE HARDWARE WARRANTY IN THESE TERMS.

#include <Wire.h>
#include <ESP32Servo.h> // Change to the standard Servo library for ESP32

Servo mot1;
Servo mot2;
Servo mot3;
Servo mot4;
const int mot1_pin = 13;
const int mot2_pin = 12;
const int mot3_pin = 14;
const int mot4_pin = 27;

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

// float Voltage, Current, BatteryRemaining, BatteryAtStart;
// float CurrentConsumed = 0;
// float BatteryDefault = 1300;

uint32_t LoopTimer;

float InputThrottle;




float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

// void battery_voltage(void)
// {
//   Voltage = (float)analogRead(15) / 62;
//   Current = (float)analogRead(21) * 0.089;
// }

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


void setup()
{
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
  
 

  pinMode(15, OUTPUT);
  digitalWrite(15, LOW);


  digitalWrite(15, HIGH);
  delay(500);
  digitalWrite(15, LOW);
  delay(500);
  digitalWrite(15, HIGH);
  delay(500);
  digitalWrite(15, LOW);
  delay(500);


  mot1.attach(mot1_pin,1000,2000);
  mot2.attach(mot2_pin,1000,2000);
  mot3.attach(mot3_pin,1000,2000);
  mot4.attach(mot4_pin,1000,2000);

  // battery_voltage();

  // if (Voltage > 8.3)
  // {
  //   digitalWrite(2, LOW);
  //   BatteryAtStart = BatteryDefault;
  // }
  // else if (Voltage < 7.5)
  // {
  //   BatteryAtStart = 30 / 100 * BatteryDefault;
  // }
  // else
  // {
  //   digitalWrite(2, LOW);
  //   BatteryAtStart = (82 * Voltage - 580) / 100 * BatteryDefault;
  // }

  // while (ReceiverValue[2] < 1020 || ReceiverValue[2] > 1050)
  // {
  //   channelInterruptHandler();
  //   delay(4);
  // }
  LoopTimer = micros();

}

void loop()
{
  InputThrottle = ReceiverValue[2];

  // if (InputThrottle > 1800)
  // {
  //   InputThrottle = 1800;
  // }
 
 //set input as reciever throttle for calibration
  MotorInput1 =  InputThrottle ;
  MotorInput2 =  InputThrottle ;
  MotorInput3 =  InputThrottle ;
  MotorInput4 =  InputThrottle ;

  //  esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 1 (front-right - CCW).
  //   esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;        //Calculate the pulse for esc 2 (rear-right - CW).
  //   esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;        //Calculate the pulse for esc 3 (rear-left - CCW).
  //   esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;

  if (MotorInput1 > 2000)
  {
    MotorInput1 = 1999;
  }

  if (MotorInput2 > 2000)
  {
    MotorInput2 = 1999;
  }

  if (MotorInput3 > 2000)
  {
    MotorInput3 = 1999;
  }

  if (MotorInput4 > 2000)
  {
    MotorInput4 = 1999;
  }


  // int ThrottleIdle = 1180;
  // if (MotorInput1 < ThrottleIdle)
  // {
  //   MotorInput1 = ThrottleIdle;
  // }
  // if (MotorInput2 < ThrottleIdle)
  // {
  //   MotorInput2 = ThrottleIdle;
  // }
  // if (MotorInput3 < ThrottleIdle)
  // {
  //   MotorInput3 = ThrottleIdle;
  // }
  // if (MotorInput4 < ThrottleIdle)
  // {
  //   MotorInput4 = ThrottleIdle;
  // }



  mot1.write(map(MotorInput1, 1000, 2000, 0, 180));
  mot2.write(map(MotorInput2, 1000, 2000, 0, 180));
  mot3.write(map(MotorInput3, 1000, 2000, 0, 180));
  mot4.write(map(MotorInput4, 1000, 2000, 0, 180));


//  battery_voltage();
//   CurrentConsumed = Current * 1000 * 0.004 / 3600 + CurrentConsumed;
//   BatteryRemaining = (BatteryAtStart - CurrentConsumed) / BatteryDefault * 100;

  // if (BatteryRemaining <= 30)
  //   digitalWrite(4, HIGH);
  // else
  //   digitalWrite(4, LOW);


// //Reciever signals
//   Serial.print(ReceiverValue[0]);
//   Serial.print(" - ");
//   Serial.print(ReceiverValue[1]);
//   Serial.print(" - ");
//   Serial.print(ReceiverValue[2]);
//   Serial.print(" - ");
//   Serial.print(ReceiverValue[3]);
//   Serial.print(" --- ");
//   // Serial.print(ReceiverValue[4]);
//   // Serial.print(" - ");
//   // Serial.print(ReceiverValue[5]);
//   // Serial.print(" - ");
// //Motor PWMs in us
//   Serial.print("  ");
//   Serial.print(MotorInput1);
//   Serial.print("  ");
//   Serial.print(MotorInput2);
//   Serial.print("  ");
//   Serial.print(MotorInput3);
//   Serial.print("  ");
//   Serial.print(MotorInput4);
//   Serial.print(" -- ");
// //Reciever translated rates
//   Serial.print(DesiredRateRoll);
//   Serial.print("  ");
//   Serial.print(DesiredRatePitch);
//   Serial.print("  ");
//   Serial.print(DesiredRateYaw);
//   Serial.print(" -- ");
// //Gyro Rates
//   Serial.print(RateRoll);
//   Serial.print("  ");
//   Serial.print(RatePitch);
//   Serial.print("  ");
//   Serial.print(RateYaw);
//   Serial.print(" -- ");
// // PID outputs
// Serial.print(InputPitch);
//   Serial.print("  ");
// Serial.print(InputRoll);
//   Serial.print("  ");
// Serial.print(InputYaw);
//   Serial.print(" -- ");


 
  while (micros() - LoopTimer < 4000);
  {
     LoopTimer = micros();
    //  Serial.print(LoopTimer);
  }

    // Serial.println("   ");

}

