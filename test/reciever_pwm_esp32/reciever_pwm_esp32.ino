// Declare variables to store the previous state of each channel and their timers
volatile uint32_t current_time;
volatile uint32_t last_channel_1 = 0, last_channel_2 = 0, last_channel_3 = 0, last_channel_4 = 0;
volatile uint32_t timer_1, timer_2, timer_3, timer_4;

volatile int receiver_input_channel[4]; // Array to store input values for 4 channels

// Define channel pin numbers
const int channel_1_pin = 34; // Pin for Channel 1
const int channel_2_pin = 35; // Pin for Channel 2
const int channel_3_pin = 32; // Pin for Channel 3
const int channel_4_pin = 33; // Pin for Channel 4

// Interrupt service routine to handle signal changes for all channels
void channelInterruptHandler()
{
  current_time = micros(); // Get the current time in microseconds

  // =================== Channel 1 ===================
  if (digitalRead(channel_1_pin))
  { // Check if Channel 1 signal is HIGH
    if (last_channel_1 == 0)
    {                         // If the previous state was LOW
      last_channel_1 = 1;     // Update state to HIGH
      timer_1 = current_time; // Record the time of the rising edge
    }
  }
  else if (last_channel_1 == 1)
  {                                                     // If the signal has gone LOW
    last_channel_1 = 0;                                 // Update state to LOW
    receiver_input_channel[0] = current_time - timer_1; // Calculate pulse width
  }

  // =================== Channel 2 ===================
  if (digitalRead(channel_2_pin))
  { // Check if Channel 2 signal is HIGH
    if (last_channel_2 == 0)
    {                         // If the previous state was LOW
      last_channel_2 = 1;     // Update state to HIGH
      timer_2 = current_time; // Record the time of the rising edge
    }
  }
  else if (last_channel_2 == 1)
  {                                                     // If the signal has gone LOW
    last_channel_2 = 0;                                 // Update state to LOW
    receiver_input_channel[1] = current_time - timer_2; // Calculate pulse width
  }

  // =================== Channel 3 ===================
  if (digitalRead(channel_3_pin))
  { // Check if Channel 3 signal is HIGH
    if (last_channel_3 == 0)
    {                         // If the previous state was LOW
      last_channel_3 = 1;     // Update state to HIGH
      timer_3 = current_time; // Record the time of the rising edge
    }
  }
  else if (last_channel_3 == 1)
  {                                                     // If the signal has gone LOW
    last_channel_3 = 0;                                 // Update state to LOW
    receiver_input_channel[2] = current_time - timer_3; // Calculate pulse width
  }

  // =================== Channel 4 ===================
  if (digitalRead(channel_4_pin))
  { // Check if Channel 4 signal is HIGH
    if (last_channel_4 == 0)
    {                         // If the previous state was LOW
      last_channel_4 = 1;     // Update state to HIGH
      timer_4 = current_time; // Record the time of the rising edge
    }
  }
  else if (last_channel_4 == 1)
  {                                                     // If the signal has gone LOW
    last_channel_4 = 0;                                 // Update state to LOW
    receiver_input_channel[3] = current_time - timer_4; // Calculate pulse width
  }
}

void print_signals()
{
  Serial.print("Channel 1 (Roll): ");
  if (receiver_input_channel[0] < 1480)
    Serial.print(" <<< ");
  else if (receiver_input_channel[0] > 1520)
    Serial.print(" >>> ");
  else
    Serial.print(" -+- ");
  Serial.print(receiver_input_channel[0]);
  Serial.print(" us  || ");

  Serial.print("Channel 2 (Nick): ");
  if (receiver_input_channel[1] < 1480)
    Serial.print(" ^^^ ");
  else if (receiver_input_channel[1] > 1520)
    Serial.print(" vvv ");
  else
    Serial.print(" -+- ");
  Serial.print(receiver_input_channel[1]);
  Serial.print(" us  || ");

  Serial.print("Channel 3 (Gas): ");
  if (receiver_input_channel[2] < 1480)
    Serial.print(" vvv ");
  else if (receiver_input_channel[2] > 1520)
    Serial.print(" ^^^ ");
  else
    Serial.print(" -+- ");
  Serial.print(receiver_input_channel[2]);
  Serial.print(" us  || ");

  Serial.print("Channel 4 (Yaw): ");
  if (receiver_input_channel[3] < 1480)
    Serial.print(" <<< ");
  else if (receiver_input_channel[3] > 1520)
    Serial.print(" >>> ");
  else
    Serial.print(" -+- ");
  Serial.println(receiver_input_channel[3]);
}

void setup()
{
  Serial.begin(115200);

  // Set channel pins as input with internal pull-up resistors
  pinMode(channel_1_pin, INPUT_PULLUP);
  pinMode(channel_2_pin, INPUT_PULLUP);
  pinMode(channel_3_pin, INPUT_PULLUP);
  pinMode(channel_4_pin, INPUT_PULLUP);

  // Attach interrupts to monitor signal changes
  attachInterrupt(digitalPinToInterrupt(channel_1_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_2_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_3_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_4_pin), channelInterruptHandler, CHANGE);
}

void loop()
{
  delay(500);
  print_signals();
}
