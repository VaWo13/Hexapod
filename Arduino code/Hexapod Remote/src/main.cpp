#include <Arduino.h>
#include <EEPROM.h>

#define PPM_Pin 6
#define PPM_frameTime 18000
#define PPM_PulseLength 300

const int transmittedVariablesCount = 5;
const int stick_deadzone = 50;

int PPM_channel = 1;

unsigned long milliseconds = 0;
unsigned long loopPeriod = 100;     //in milliseconds

float transmitted_variables[transmittedVariablesCount];
float channels[8];
float raw_Input_channels[4];
float Input_channels[4];
float Output_channels[8];
int channel_order[8] = {5, 4, 2, 3, 1, 0, 6, 7};

int Analog_Sticks_Cal[4][3] = 
{
  EEPROM.read(0)  | EEPROM.read(1) << 8,  EEPROM.read(2)  | EEPROM.read(3) << 8,  EEPROM.read(4)  | EEPROM.read(5) << 8,  // R LR   Aileron
  EEPROM.read(6)  | EEPROM.read(7) << 8,  EEPROM.read(8)  | EEPROM.read(9) << 8,  EEPROM.read(10) | EEPROM.read(11) << 8, // R UD   Elevator
  EEPROM.read(12) | EEPROM.read(13) << 8, EEPROM.read(14) | EEPROM.read(15) << 8, EEPROM.read(16) | EEPROM.read(17) << 8, // L UD   Thottle
  EEPROM.read(18) | EEPROM.read(19) << 8, EEPROM.read(20) | EEPROM.read(21) << 8, EEPROM.read(22) | EEPROM.read(23) << 8, // L LR   Rudder
};//min mid max

long realFrameTime = 0;

bool PPM_pusle = true;
bool calibration = true;
bool Stick_Cal_start = true;
bool pastButton_1;
bool pastButton_2;
bool pastButton_3;
bool pastButton_4;

bool timeIsUp();
void get_input_values();
void send_movementInfo();
void get_input_channels();
void Channel_mixing();
void PPM_trainer_port();
void Stick_Cal_Mode();


void setup() {
  // put your setup code here, to run once:

  pinMode(PPM_Pin, OUTPUT);

  noInterrupts();

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = PPM_frameTime;  //compare register A
  TCCR1B = 0b00000010;  //8 - prescaler for timer 1 (0,5 us per tick)
  TIMSK1 = 0b00000010;    //output mask

  interrupts();
  digitalWrite(PPM_Pin, LOW);

  Serial.begin(9600);
  Serial.setTimeout(1);

  Serial.println("Reset");
}

void loop() {
  // put your main code here, to run repeatedly:
  if (timeIsUp())
  {
    get_input_values();
    //send_movementInfo();
  }
}

ISR(TIMER1_COMPA_vect)
{
  PPM_trainer_port();
}

bool timeIsUp()
{
  bool timeIsUp = false;
  if ((millis() - milliseconds) >= loopPeriod)
  {
    milliseconds = millis();
    timeIsUp = true;
  }
  return timeIsUp;
}

void get_input_values()
{

  // channels[0] = map((analogRead(5) + 0), 0, 1023, 1000, 2000);
  // channels[1] = map((analogRead(4) + 0), 1023, 0, 1000, 2000);
  // channels[2] = map((analogRead(2) + 0), 1023, 0, 1000, 2000);
  // channels[3] = map((analogRead(3) + 0), 0, 1023, 1000, 2000);
  // channels[4] = map((analogRead(1) + 0), 0, 1023, 1000, 2000);
  // channels[5] = map((analogRead(0) + 0), 0, 1023, 1000, 2000);
  // channels[6] = map((analogRead(6) + 0), 0, 1023, 1000, 2000);
  // channels[7] = map((analogRead(7) + 0), 0, 1023, 1000, 2000);



  // transmitted_variables[0] = (analogRead(5) - 512) * -1; //J1 x    //crab sideways
  // transmitted_variables[1] = analogRead(4) - 512;        //J1 y    //forwar/backward
  // transmitted_variables[2] = analogRead(1) - 512;        //J2 x    //rotate left/right
  // transmitted_variables[3] = analogRead(0) - 512;        //J2 y    //no
  // transmitted_variables[5] = analogRead(7);              //J3 x    //no
  // transmitted_variables[4] = analogRead(6);              //J3 y    //no
  // transmitted_variables[7] = analogRead(3);              //J4 x    //no
  // transmitted_variables[6] = analogRead(2);              //J4 y    //bodyheight
  // transmitted_variables[8] = digitalRead(3);             //B1
  // transmitted_variables[9] = digitalRead(2);             //B2
  // transmitted_variables[10] = digitalRead(4);            //B3
  // transmitted_variables[11] = digitalRead(5);            //B4

  bool button_1 = digitalRead(3);

  transmitted_variables[0] = (analogRead(4) - 512) * -1;        //J1 y    //forwar/backward
  transmitted_variables[1] = (analogRead(5) - 512) *  1;        //J1 x    //crab sideways
  transmitted_variables[2] = (analogRead(1) - 512 )* -1;        //J2 x    //rotate left/right
  transmitted_variables[3] = 1023 - analogRead(2);              //J4 y    //bodyheight

  // Serial.println(transmitted_variables[0]);
  // Serial.println(transmitted_variables[1]);
  // Serial.println(transmitted_variables[2]);
  // Serial.println(transmitted_variables[3]);

  //stick deadzones
  for (size_t i = 0; i < 3; i++)
  {
    if ((transmitted_variables[i] < stick_deadzone) & (transmitted_variables[i] > (stick_deadzone * -1)))
    {
      transmitted_variables[i] = 0;
    }
  }
  
  //button edge detection

  if ((pastButton_1 == HIGH) & (button_1 == LOW)) //B1 Rising edge
  { 
    calibration = calibration ^ true; // "^"(XOR) toggles bool variable
  }
  transmitted_variables[4] = calibration;
  pastButton_1 = button_1;

  //Stick calibration

  if (digitalRead(5))
  {
    Stick_Cal_Mode();
  }
  else
  {
    if (Stick_Cal_start == false)
    {
      Serial.println("EEPROM Write");
      for (size_t i = 0; i < 4; i++)
      {
        EEPROM.write(0 + (6 * i), uint8_t(Analog_Sticks_Cal[i][0] & 0xFF));
        EEPROM.write(1 + (6 * i), uint8_t(Analog_Sticks_Cal[i][0] >> 8));
        EEPROM.write(2 + (6 * i), uint8_t(Analog_Sticks_Cal[i][1] & 0xFF));
        EEPROM.write(3 + (6 * i), uint8_t(Analog_Sticks_Cal[i][1] >> 8));
        EEPROM.write(4 + (6 * i), uint8_t(Analog_Sticks_Cal[i][2] & 0xFF));
        EEPROM.write(5 + (6 * i), uint8_t(Analog_Sticks_Cal[i][2] >> 8));
      }
      
    }
    
    Stick_Cal_start = true; 
  }

}

void send_movementInfo()
{
  // if (transmitted_variables[8] == 0)
  // {
  //   Serial.print("C f");
  // }
  // delay(20);
  // Serial.print(";");
  // for (size_t i = 0; i < 4; i++)
  // {
  //   Serial.print(transmitted_variables[i]);
  //   Serial.print(";");
  // }
  // Serial.println("");

  Serial.print('[');  //Start byte
  for (size_t i = 0; i < transmittedVariablesCount; i++)  //send all variables
  {
    Serial.print(transmitted_variables[i]);
    Serial.print('/');
  }
  Serial.println(']');  //End byte
}

void get_input_channels()
{
  raw_Input_channels[0] = analogRead(5);//map(analogRead(5), 0, 1023, -100,  100) + 4.5;
  raw_Input_channels[1] = analogRead(4);//map(analogRead(4), 0, 1023,  100, -100) + 2;
  raw_Input_channels[2] = analogRead(2);//map(analogRead(2), 80, 865,  100, -100);
  raw_Input_channels[3] = analogRead(3);//map(analogRead(3), 0, 1023, -100,  100) - 2;

  for (size_t i = 0; i < 4; i++)
  {
    if (raw_Input_channels[i] == Analog_Sticks_Cal[i][1])
    {
      Input_channels[i] = 0;
    }
    else
    {
      if (raw_Input_channels[i] < Analog_Sticks_Cal[i][1])
      {
        Input_channels[i] = map(raw_Input_channels[i], Analog_Sticks_Cal[i][0], Analog_Sticks_Cal[i][1], -100, 0);
      }   
      else
      {
        Input_channels[i] = map(raw_Input_channels[i], Analog_Sticks_Cal[i][1], Analog_Sticks_Cal[i][2], 0, 100);
      }   
    }
  }
}

void Channel_mixing()
{
  Output_channels[0] = Input_channels[0];
  Output_channels[1] = Input_channels[1];
  Output_channels[2] = Input_channels[2];
  Output_channels[3] = Input_channels[3];

  for (size_t i = 0; i < 8; i++)                            //Limit Channel range
  {
    if (Output_channels[i] > 100) Output_channels[i] = 100;
    if (Output_channels[i] < -100) Output_channels[i] = -100;
  }
}

void PPM_trainer_port()
{
  digitalWrite(PPM_Pin, digitalRead(PPM_Pin) ^ 1);  //toggle pin

  if (PPM_pusle == true)
  {
    TCNT1 = 0;
    PPM_pusle = false;
    OCR1A = (PPM_PulseLength - 14) * 2;  //compare register A
  }
  else
  {
    if (PPM_channel < 9)
    {

      // channels[PPM_channel - 1] = map((analogRead(channel_order[PPM_channel - 1]) + 0), 1023, 0, 1000, 2000);
      OCR1A = (map(Output_channels[PPM_channel - 1], -100, 100, 1000, 2000) - 14) * 2; //compare register A
      realFrameTime += map(Output_channels[PPM_channel - 1], -100, 100, 1000, 2000);
      PPM_pusle = true;
    }
    else
    {
      OCR1A = (PPM_frameTime - realFrameTime - 14) * 2;  //sync time
      
      get_input_channels();
      Channel_mixing();

      PPM_pusle = true;
      PPM_channel = 0;
      realFrameTime = 0;
    }
    PPM_channel ++;
  }
}

void Stick_Cal_Mode()
{
  if (Stick_Cal_start)
  {   //set center Position on first pass
    Analog_Sticks_Cal[0][1] = raw_Input_channels[0];
    Analog_Sticks_Cal[1][1] = raw_Input_channels[1];
    Analog_Sticks_Cal[2][1] = raw_Input_channels[2];
    Analog_Sticks_Cal[3][1] = raw_Input_channels[3];

    for (size_t i = 0; i < 4; i++)
    {
      Analog_Sticks_Cal[i][0] = Analog_Sticks_Cal[i][1];    //reset min to center
      Analog_Sticks_Cal[i][2] = Analog_Sticks_Cal[i][1];    //reset max to center
    }
    
    Stick_Cal_start = false;
  }
  else
  {
    for (size_t i = 0; i < 4; i++)
    {
      if (raw_Input_channels[i] < Analog_Sticks_Cal[i][0]) Analog_Sticks_Cal[i][0] = raw_Input_channels[i];   //adjust min
      if (raw_Input_channels[i] > Analog_Sticks_Cal[i][2]) Analog_Sticks_Cal[i][2] = raw_Input_channels[i];   //adjust max
    } 
  }
}