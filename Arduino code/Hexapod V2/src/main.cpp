#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include "PCA9685.h"

unsigned long milliseconds = 0;
unsigned long loopPeriod = 25;     //in milliseconds

bool calibration_servos = true;
float Body_moveVectorComponent[3] = {0, 0, 0};    //x, y, angle  in mm/deg   distance/angle per calculated step
float body_height = 30;                           //height from floor to corepoint in mm
float height_liftArms = 20;                       //max height of arms in lift mode in mm

//corepoint is always at 0/0/0 at angles 0°/0°/0°
//floor moves (P2)

PCA9685 pwm_Board;

PCA9685_ServoEvaluator servo_Arm1_alpha;
PCA9685_ServoEvaluator servo_Arm1_beta;
PCA9685_ServoEvaluator servo_Arm1_gamma;

PCA9685_ServoEvaluator servo_Arm2_alpha;
PCA9685_ServoEvaluator servo_Arm2_beta;
PCA9685_ServoEvaluator servo_Arm2_gamma;

PCA9685_ServoEvaluator servo_Arm3_alpha;
PCA9685_ServoEvaluator servo_Arm3_beta;
PCA9685_ServoEvaluator servo_Arm3_gamma;

PCA9685_ServoEvaluator servo_Arm4_alpha;
PCA9685_ServoEvaluator servo_Arm4_beta;
PCA9685_ServoEvaluator servo_Arm4_gamma;

PCA9685_ServoEvaluator servo_Arm5_alpha;
PCA9685_ServoEvaluator servo_Arm5_beta;
PCA9685_ServoEvaluator servo_Arm5_gamma;

PCA9685_ServoEvaluator servo_Arm6_alpha;
Servo servo_Arm6_beta; 
Servo servo_Arm6_gamma;



// variables for all arms
float Arm[6][17] = {
  { 40,  30, body_height,  45,  44, body_height - 20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {  0,  30, body_height,   0,  44, body_height - 20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {-40,  30, body_height, -45,  44, body_height - 20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  { 40, -30, body_height,  45, -44, body_height - 20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {  0, -30, body_height,   0, -44, body_height - 20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
  {-40, -30, body_height, -45, -44, body_height - 20, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};

// all constants
const float Arm_c[6][11] = {                            
//{  A,   B, r_T,  x_T, y_T, alpha_off, beta_off, gamma_off, alpha_inv, beta_inv, gamma_inv}
  { 50,  60,  20,   80,  70,        -7,       -4,         5,        -1,       -1,        -1},
  { 50,  60,  20,    0,  95,        -3,/*-7*/-11,        -3,        -1,       -1,        -1},
  { 50,  60,  20,  -80,  70,         5,       -2,        -6,        -1,       -1,        -1},
  { 50,  60,  20,   80, -70,        -4,       -9,        -1,        -1,        1,         1},
  { 50,  60,  20,    0, -95,         6,       -1,        -1,        -1,        1,         1},
  { 50,  60,  20,  -80, -70,         6,      -10,        -6,        -1,        1,         1},
};

bool Arm_lift[6] = {1, 0, 1, 0, 1, 0};

bool readingInProgress;
bool readingDone;

bool timeleftover;

char queued_byte;
String recieved_string;

const int transmittedVariablesCount = 5;

bool timeIsUp();

void getMove();
void getAngles();
void updateServos();
void getmovementinfo();

void setup()
{
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  Wire.begin();
  Wire.setClock(400000);
  pwm_Board.resetDevices();
  pwm_Board.init(B000000);
  pwm_Board.setPWMFrequency(50);

  servo_Arm6_beta.attach(9);
  servo_Arm6_gamma.attach(10);

  Serial.begin(9600);
  Serial.setTimeout(10);

  Serial.println("Reset");
  
  Arm[0][6] = 0;  //alpha
  Arm[0][7] = 0;  //beta
  Arm[0][8] = 0;  //gamma
  Arm[1][6] = 0;  //alpha
  Arm[1][7] = 0;  //beta
  Arm[1][8] = 0;  //gamma
  Arm[2][6] = 0;  //alpha
  Arm[2][7] = 0;  //beta
  Arm[2][8] = 0;  //gamma
  Arm[3][6] = 0;  //alpha
  Arm[3][7] = 0;  //beta
  Arm[3][8] = 0;  //gamma
  Arm[4][6] = 0;  //alpha
  Arm[4][7] = 0;  //beta
  Arm[4][8] = 0;  //gamma
  Arm[5][6] = 0;  //alpha
  Arm[5][7] = 0;  //beta
  Arm[5][8] = 0;  //gamma
  
  updateServos();
}

void loop()
{
  // put your main code here, to run repeatedly:
  if (timeIsUp())
  {
    timeleftover = false;
    getmovementinfo();

    if (calibration_servos == false)
    {
      if ((Body_moveVectorComponent[0] != 0) | (Body_moveVectorComponent[1] != 0) | (Body_moveVectorComponent[2] != 0))
      {
        getMove();
      }
      getAngles();
    }
    updateServos();
  }
}

bool timeIsUp()
{
  bool timeIsUp = false;
  if ((millis() - milliseconds) >= loopPeriod)
  {
    milliseconds = millis();
    timeIsUp = true;
    
    if (timeleftover == false)
    {
      digitalWrite(13, HIGH);
    }
    else
    {
      digitalWrite(13, LOW);
    }
  }
  else
  {
    timeleftover = true;
  }
  
  return timeIsUp;
}


void getMove()
{
  float d_Min_liftspeed_A1A3A5 = 0;
  float d_moved_A1A3A5 = 0;
  float d_Min_liftspeed_A2A4A6 = 0;
  float d_moved_A2A4A6 = 0;

  //create recovery pos x/y
  for (size_t i = 0; i < 6; i++)
  {
    Arm[i][9] = Arm[i][3];
    Arm[i][10] = Arm[i][4]; 
  }
  //new P2 and d_remaining for Arms which are on floor
  for (size_t i = 0; i < 6; i++)
  {
    if (Arm_lift[i] == false)
    {
      float r_temp = sqrt(((Arm[i][3] - Body_moveVectorComponent[0]) * (Arm[i][3] - Body_moveVectorComponent[0])) + ((Arm[i][4] - Body_moveVectorComponent[1]) * (Arm[i][4] - Body_moveVectorComponent[1]))); //rotate 
      float angle_temp = (asin((Arm[i][3] - Body_moveVectorComponent[0]) / r_temp) * 180) / PI;
      if (i < 3)    //apply reversed Body_moveVector to P2
      {
        Arm[i][3] = (sin(((angle_temp + Body_moveVectorComponent[2]) * PI) / 180) * r_temp);
        Arm[i][4] = (cos(((angle_temp + Body_moveVectorComponent[2]) * PI) / 180) * r_temp);
      }
      else
      {
        Arm[i][3] = (sin(((angle_temp - Body_moveVectorComponent[2]) * PI) / 180) * r_temp);
        Arm[i][4] = (cos(((angle_temp - Body_moveVectorComponent[2]) * PI) / 180) * -r_temp);   //watch out for "-" on r_temp
      }
      //d_remaining
      float r_recovery_P2 = sqrt(((Arm_c[i][3] - Arm[i][9]) * (Arm_c[i][3] - Arm[i][9])) + ((Arm_c[i][4] - Arm[i][10]) * (Arm_c[i][4] - Arm[i][10])));
      Arm[i][12] = sqrt(((Arm[i][3] - Arm[i][9]) * (Arm[i][3] - Arm[i][9])) + ((Arm[i][4] - Arm[i][10]) * (Arm[i][4] - Arm[i][10])));
      float cosbeta_temp;
      if (r_recovery_P2 != 0)
      {
        cosbeta_temp = ( ( (r_recovery_P2 * r_recovery_P2) + (Arm[i][12] * Arm[i][12]) - (((Arm_c[i][3] - Arm[i][3]) * (Arm_c[i][3] - Arm[i][3])) + ((Arm_c[i][4] - Arm[i][4]) * (Arm_c[i][4] - Arm[i][4]))) ) / (2 * r_recovery_P2 * Arm[i][12]) );
        if (cosbeta_temp > 1)
        {
          cosbeta_temp = 1;
        }
        if (cosbeta_temp < -1)
        {
          cosbeta_temp = -1;
        }
      }
      else
      {
        cosbeta_temp = -1;
      }
      float beta_temp = acos(cosbeta_temp);
      Arm[i][11] = sqrt((r_recovery_P2 * r_recovery_P2) + (Arm_c[i][2] * Arm_c[i][2]) - (2 * r_recovery_P2 * Arm_c[i][2] * cos((180 - ((beta_temp * 180) / PI)) - ((asin((r_recovery_P2 / Arm_c[i][2]) * sin(beta_temp)) * 180) / PI)))); 
    }
  }
  //check for shortest d_remaining for triangle 1 (arm 1 3 5)
  if ((Arm_lift[0] == false) & (Arm_lift[2] == false) & (Arm_lift[4] == false))
  {
    if (Arm[0][11] < Arm[2][11])
    {
      if (Arm[0][11] < Arm[4][11])
      {
        d_Min_liftspeed_A1A3A5 = Arm[0][11];
        d_moved_A1A3A5 = sqrt(((Arm[0][3] - Arm[0][9]) * (Arm[0][3] - Arm[0][9])) + ((Arm[0][4] - Arm[0][10]) * (Arm[0][4] - Arm[0][10])));
      }
      else
      {
        d_Min_liftspeed_A1A3A5 = Arm[4][11];
        d_moved_A1A3A5 = sqrt(((Arm[4][3] - Arm[4][9]) * (Arm[4][3] - Arm[4][9])) + ((Arm[4][4] - Arm[4][10]) * (Arm[4][4] - Arm[4][10])));
      }
      
    }
    else
    {
      if (Arm[2][11] < Arm[4][11])
      {
        d_Min_liftspeed_A1A3A5 = Arm[2][11];
        d_moved_A1A3A5 = sqrt(((Arm[2][3] - Arm[2][9]) * (Arm[2][3] - Arm[2][9])) + ((Arm[2][4] - Arm[2][10]) * (Arm[2][4] - Arm[2][10])));
      }
      else
      {
        d_Min_liftspeed_A1A3A5 = Arm[4][11];
        d_moved_A1A3A5 = sqrt(((Arm[4][3] - Arm[4][9]) * (Arm[4][3] - Arm[4][9])) + ((Arm[4][4] - Arm[4][10]) * (Arm[4][4] - Arm[4][10])));
      }
      
    }
  }
  //check for shortest d_remaining for triangle 2 (arm 2 4 6)
  if ((Arm_lift[1] == false) & (Arm_lift[3] == false) & (Arm_lift[5] == false))
  {
    if (Arm[1][11] < Arm[3][11])
    {
      if (Arm[1][11] < Arm[5][11])
      {
        d_Min_liftspeed_A2A4A6 = Arm[1][11];
        d_moved_A2A4A6 = sqrt(((Arm[1][3] - Arm[1][9]) * (Arm[1][3] - Arm[1][9])) + ((Arm[1][4] - Arm[1][10]) * (Arm[1][4] - Arm[1][10])));
      }
      else
      {
        d_Min_liftspeed_A2A4A6 = Arm[5][11];
        d_moved_A2A4A6 = sqrt(((Arm[5][3] - Arm[5][9]) * (Arm[5][3] - Arm[5][9])) + ((Arm[5][4] - Arm[5][10]) * (Arm[5][4] - Arm[5][10])));
      }
    }
    else
    {
      if (Arm[3][11] < Arm[5][11])
      {
        d_Min_liftspeed_A2A4A6 = Arm[3][11];
        d_moved_A2A4A6 = sqrt(((Arm[3][3] - Arm[3][9]) * (Arm[3][3] - Arm[3][9])) + ((Arm[3][4] - Arm[3][10]) * (Arm[3][4] - Arm[3][10])));
      }
      else
      {
        d_Min_liftspeed_A2A4A6 = Arm[5][11];
        d_moved_A2A4A6 = sqrt(((Arm[5][3] - Arm[5][9]) * (Arm[5][3] - Arm[5][9])) + ((Arm[5][4] - Arm[5][10]) * (Arm[5][4] - Arm[5][10])));
      }
    }
  }
  //get target point/vector, movement speed and apply to Arms
  for (size_t i = 0; i < 6; i++)
  {
    if (Arm_lift[i] == true)
    {
      //arm in lift mode -- moves in direction of target point at which the arm touches floor again

      float P_target_move[2] = {0, 0};
      float P_target[2] = {0, 0};

      float r_temp = sqrt(((Arm_c[i][3] + Body_moveVectorComponent[0]) * (Arm_c[i][3] + Body_moveVectorComponent[0])) + ((Arm_c[i][4] + Body_moveVectorComponent[1]) * (Arm_c[i][4] + Body_moveVectorComponent[1]))); //rotate 
      float angle_temp = (asin((Arm_c[i][3] + Body_moveVectorComponent[0]) / r_temp) * 180) / PI;
      if (i < 3)    //calculate target Point
      {
        P_target_move[0] = (sin(((angle_temp - Body_moveVectorComponent[2]) * PI) / 180) * r_temp) - Arm_c[i][3];
        P_target_move[1] = (cos(((angle_temp - Body_moveVectorComponent[2]) * PI) / 180) * r_temp) - Arm_c[i][4];
      }
      else
      {
        P_target_move[0] = (sin(((angle_temp + Body_moveVectorComponent[2]) * PI) / 180) * r_temp) - Arm_c[i][3];
        P_target_move[1] = (cos(((angle_temp + Body_moveVectorComponent[2]) * PI) / 180) * -r_temp) - Arm_c[i][4];   //watch out for "-" on r_temp
      }

      float r_P_target_move = sqrt((P_target_move[0] * P_target_move[0]) + (P_target_move[1] * P_target_move[1]));

      if (r_P_target_move != 0)   // get P_target x/y
      {
        P_target[0] = Arm_c[i][3] + ((Arm_c[i][2] * P_target_move[0]) / r_P_target_move);
        P_target[1] = Arm_c[i][4] + ((Arm_c[i][2] * P_target_move[1]) / r_P_target_move);
      }
      else
      {
        P_target[0] = Arm_c[i][3];
        P_target[0] = Arm_c[i][4];
      }

      float V_P2Pt[2] = {P_target[0] - Arm[i][3], P_target[1] - Arm[i][4]}; //  x/y between P2 and P_target

      Arm[i][13] = sqrt((V_P2Pt[0] * V_P2Pt[0]) + (V_P2Pt[1] * V_P2Pt[1]));   //distance between P2 and P_target

      if ((V_P2Pt[0] != 0) | (V_P2Pt[1] != 0))
      {
        float liftspeed_temp;              
        if ((i == 0) | (i == 2) | (i == 4))     //calculate movement speed for Arms 1 3 5
        {
          if (d_Min_liftspeed_A2A4A6 == 0)
          {
            liftspeed_temp = r_P_target_move;
          }
          else
          {
            liftspeed_temp = ((d_moved_A2A4A6 * Arm[i][13]) / d_Min_liftspeed_A2A4A6);
          }
        }
        else                                    //calculate movement speed for Arms 2 4 6
        {
          if (d_Min_liftspeed_A1A3A5 == 0)
          {
            liftspeed_temp = r_P_target_move;
          }
          else
          {
            liftspeed_temp = ((d_moved_A1A3A5 * Arm[i][13]) / d_Min_liftspeed_A1A3A5);
          }
        }
        
        Arm[i][3] = Arm[i][3] + ((liftspeed_temp * V_P2Pt[0]) / (Arm[i][13]));
        Arm[i][4] = Arm[i][4] + ((liftspeed_temp * V_P2Pt[1]) / (Arm[i][13]));
        //get Arm height
        // Arm[i][5] = (height_liftArms * (Arm_c[i][2] - sqrt(((Arm_c[i][3] - Arm[i][3]) * (Arm_c[i][3] - Arm[i][3])) + ((Arm_c[i][4] - Arm[i][4]) * (Arm_c[i][4] - Arm[i][4]))))) / Arm_c[i][2];
      }
    }
  }
  //set height for lift Arms
  if ((Arm_lift[0] == true) & (Arm_lift[2] == true) & (Arm_lift[4] == true))
  {
    int   d_shortest_A1A3A5 = 0;
    float height_d_shortest = 0;

    //get shortest d_remaining
    if (Arm[0][13] < Arm[2][13])
    {
      if (Arm[0][13] < Arm[4][13])
      {
        d_shortest_A1A3A5 = 0;
      }
      else
      {
        d_shortest_A1A3A5 = 4;
      }
    }
    else
    {
      if (Arm[2][13] < Arm[4][13])
      {
        d_shortest_A1A3A5 = 2;
      }
      else
      {
        d_shortest_A1A3A5 = 4;
      }
    }
    //calculate height for Arm with shortest d_remaining
    height_d_shortest = (height_liftArms * (Arm_c[d_shortest_A1A3A5][2] - sqrt(((Arm_c[d_shortest_A1A3A5][3] - Arm[d_shortest_A1A3A5][3]) * (Arm_c[d_shortest_A1A3A5][3] - Arm[d_shortest_A1A3A5][3])) + ((Arm_c[d_shortest_A1A3A5][4] - Arm[d_shortest_A1A3A5][4]) * (Arm_c[d_shortest_A1A3A5][4] - Arm[d_shortest_A1A3A5][4]))))) / Arm_c[d_shortest_A1A3A5][2];
    //set Armheights
    Arm[0][5] = height_d_shortest;
    Arm[2][5] = height_d_shortest;
    Arm[4][5] = height_d_shortest;
  }
  else
  {
    int   d_shortest_A2A4A6 = 0;
    float height_d_shortest = 0;

    //get shortest d_remaining
    if (Arm[1][13] < Arm[3][13])
    {
      if (Arm[1][13] < Arm[5][13])
      {
        d_shortest_A2A4A6 = 1;
      }
      else
      {
        d_shortest_A2A4A6 = 5;
      }
    }
    else
    {
      if (Arm[3][13] < Arm[5][13])
      {
        d_shortest_A2A4A6 = 3;
      }
      else
      {
        d_shortest_A2A4A6 = 5;
      }
    }
    //calculate height for Arm with shortest d_remaining
    height_d_shortest = (height_liftArms * (Arm_c[d_shortest_A2A4A6][2] - sqrt(((Arm_c[d_shortest_A2A4A6][3] - Arm[d_shortest_A2A4A6][3]) * (Arm_c[d_shortest_A2A4A6][3] - Arm[d_shortest_A2A4A6][3])) + ((Arm_c[d_shortest_A2A4A6][4] - Arm[d_shortest_A2A4A6][4]) * (Arm_c[d_shortest_A2A4A6][4] - Arm[d_shortest_A2A4A6][4]))))) / Arm_c[d_shortest_A2A4A6][2];
    //set Armheights
    Arm[1][5] = height_d_shortest;
    Arm[3][5] = height_d_shortest;
    Arm[5][5] = height_d_shortest;
  }
  //check if "lift Arms" touched floor or "floor arms" reached tolerance circle
  for (size_t i = 0; i < 6; i++)
  {
    if (Arm_lift[i] == false)
    {
      if (sqrt(((Arm_c[i][3] - Arm[i][3]) * (Arm_c[i][3] - Arm[i][3])) + ((Arm_c[i][4] - Arm[i][4]) * (Arm_c[i][4] - Arm[i][4]))) > Arm_c[i][2])  //if outside tolerance circle
      {
        if ((i == 1) | (i == 3) | (i == 5))   //set for Arms 2 4 6
        {
          //change lift mode
          Arm_lift[1] = true;
          Arm_lift[3] = true;
          Arm_lift[5] = true;
          //set P2 to recovery x/y and adjust height
          Arm[1][3] = Arm[1][9];
          Arm[1][4] = Arm[1][10];
          Arm[1][5] = 1;
          Arm[3][3] = Arm[3][9];
          Arm[3][4] = Arm[3][10];
          Arm[3][5] = 1;
          Arm[5][3] = Arm[5][9];
          Arm[5][4] = Arm[5][10];
          Arm[5][5] = 1;

          Arm_lift[0] = false;
          Arm_lift[2] = false;
          Arm_lift[4] = false;

          Arm[0][3] = Arm[0][9];
          Arm[0][4] = Arm[0][10];
          Arm[0][5] = 0;
          Arm[2][3] = Arm[2][9];
          Arm[2][4] = Arm[2][10];
          Arm[2][5] = 0;
          Arm[4][3] = Arm[4][9];
          Arm[4][4] = Arm[4][10];
          Arm[4][5] = 0;

        }
        else                                  //set for Arms 1 3 5
        {
          //change lift mode
          Arm_lift[0] = true;
          Arm_lift[2] = true;
          Arm_lift[4] = true;
          //set P2 to recovery x/y and adjust height
          Arm[0][3] = Arm[0][9];
          Arm[0][4] = Arm[0][10];
          Arm[0][5] = 1;
          Arm[2][3] = Arm[2][9];
          Arm[2][4] = Arm[2][10];
          Arm[2][5] = 1;
          Arm[4][3] = Arm[4][9];
          Arm[4][4] = Arm[4][10];
          Arm[4][5] = 1;

          Arm_lift[1] = false;
          Arm_lift[3] = false;
          Arm_lift[5] = false;

          Arm[1][3] = Arm[1][9];
          Arm[1][4] = Arm[1][10];
          Arm[1][5] = 0;
          Arm[3][3] = Arm[3][9];
          Arm[3][4] = Arm[3][10];
          Arm[3][5] = 0;
          Arm[5][3] = Arm[5][9];
          Arm[5][4] = Arm[5][10];
          Arm[5][5] = 0;
        }
      }
    }
  }
}
void getAngles()
{
  for (size_t i = 0; i < 6; i++)
  {
    float lpp = sqrt(((Arm[i][0] - Arm[i][3]) * (Arm[i][0] - Arm[i][3])) + ((Arm[i][1] - Arm[i][4]) * (Arm[i][1] - Arm[i][4])) + ((Arm[i][2] - Arm[i][5]) * (Arm[i][2] - Arm[i][5])));
    Arm[i][6] = (atan((Arm[i][0] - Arm[i][3]) / (Arm[i][1] - Arm[i][4])) * 180) / PI;
    float epsilon = (asin((Arm[i][5] - Arm[i][2]) / lpp) * 180) / PI;
    float delta = acos(((Arm_c[i][0] * Arm_c[i][0]) + (lpp * lpp) - (Arm_c[i][1] * Arm_c[i][1])) / (2 * Arm_c[i][0] * lpp));
    Arm[i][8] = ((acos(((Arm_c[i][0] * Arm_c[i][0]) + (Arm_c[i][1] * Arm_c[i][1]) - (lpp * lpp)) / (2 * Arm_c[i][0] * Arm_c[i][1])) * 180) / PI) - 90;
    Arm[i][7] = (((delta * 180) / PI) + epsilon);
  }
  
}
void updateServos()
{
  for (size_t i = 0; i < 6; i++)  //Smooth servo movement
  {
    Arm[i][6] = Arm[i][14] + ((Arm[i][6] - Arm[i][14]) / 5);
    Arm[i][7] = Arm[i][15] + ((Arm[i][7] - Arm[i][15]) / 5);
    Arm[i][8] = Arm[i][16] + ((Arm[i][8] - Arm[i][16]) / 5);

    Arm[i][14] = Arm[i][6];   //set current servo angle as old
    Arm[i][15] = Arm[i][7];
    Arm[i][16] = Arm[i][8];
  }
  
    pwm_Board.setChannelPWM(0,  servo_Arm1_alpha.pwmForAngle((Arm[0][6] + Arm_c[0][5]) * Arm_c[0][8]));
    pwm_Board.setChannelPWM(1,   servo_Arm1_beta.pwmForAngle((Arm[0][7] + Arm_c[0][6]) * Arm_c[0][9]));
    pwm_Board.setChannelPWM(2,  servo_Arm1_gamma.pwmForAngle((Arm[0][8] + Arm_c[0][7]) * Arm_c[0][10]));
  
    pwm_Board.setChannelPWM(6,  servo_Arm2_alpha.pwmForAngle((Arm[1][6] + Arm_c[1][5]) * Arm_c[1][8]));
    pwm_Board.setChannelPWM(7,   servo_Arm2_beta.pwmForAngle((Arm[1][7] + Arm_c[1][6]) * Arm_c[1][9]));
    pwm_Board.setChannelPWM(8,  servo_Arm2_gamma.pwmForAngle((Arm[1][8] + Arm_c[1][7]) * Arm_c[1][10]));
  
    pwm_Board.setChannelPWM(12, servo_Arm3_alpha.pwmForAngle((Arm[2][6] + Arm_c[2][5]) * Arm_c[2][ 8]));
    pwm_Board.setChannelPWM(13,  servo_Arm3_beta.pwmForAngle((Arm[2][7] + Arm_c[2][6]) * Arm_c[2][ 9]));
    pwm_Board.setChannelPWM(14, servo_Arm3_gamma.pwmForAngle((Arm[2][8] + Arm_c[2][7]) * Arm_c[2][10]));
    
    pwm_Board.setChannelPWM(3,  servo_Arm4_alpha.pwmForAngle((Arm[3][6] + Arm_c[3][5]) * Arm_c[3][ 8]));
    pwm_Board.setChannelPWM(4,   servo_Arm4_beta.pwmForAngle((Arm[3][7] + Arm_c[3][6]) * Arm_c[3][ 9]));
    pwm_Board.setChannelPWM(5,  servo_Arm4_gamma.pwmForAngle((Arm[3][8] + Arm_c[3][7]) * Arm_c[3][10]));
  
    pwm_Board.setChannelPWM(9,  servo_Arm5_alpha.pwmForAngle((Arm[4][6] + Arm_c[4][5]) * Arm_c[4][ 8]));
    pwm_Board.setChannelPWM(10,  servo_Arm5_beta.pwmForAngle((Arm[4][7] + Arm_c[4][6]) * Arm_c[4][ 9]));
    pwm_Board.setChannelPWM(11, servo_Arm5_gamma.pwmForAngle((Arm[4][8] + Arm_c[4][7]) * Arm_c[4][10]));
  
    pwm_Board.setChannelPWM(15, servo_Arm6_alpha.pwmForAngle((Arm[5][6] + Arm_c[5][5]) * Arm_c[5][ 8]));  //PCA9685.h uses angles (-90) - 90°
                                   servo_Arm6_beta.write((Arm[5][7] + Arm_c[5][6] + 90) * Arm_c[5][ 9]);  //Servo.h uses angles       0 - 180°
                                  servo_Arm6_gamma.write((Arm[5][8] + Arm_c[5][7] + 90) * Arm_c[5][10]);
}
void getmovementinfo()
{
  while (Serial.available() > 0 )
  {
    queued_byte = Serial.read();

    //adds queued bytes to recieved_string and confirms its finished
    if (queued_byte != '[') //Start byte
    {
      if (readingInProgress == true)
      {
        if (queued_byte != ']') //End byte
        {
          recieved_string += queued_byte; //add to string
        }
        else
        {
          readingInProgress = false; 
          readingDone = true; //confirm finished
        }
      }
    }
    else
    {
      readingInProgress = true; //start reading
    }

    //convert string to individual variables
    if (readingDone)
    {
      readingDone = false;    //reset filter

      String recieved_variables[transmittedVariablesCount];
      int tempBytePos = 0;

      //divide recieved string into individual variable strings  
      for (size_t i = 0; i < transmittedVariablesCount; i++)
      {
        while (recieved_string[tempBytePos] != '/')
        {
          recieved_variables[i] += recieved_string[tempBytePos];
          tempBytePos ++;
        }
        tempBytePos ++;
      }

      //set all variables
      if (calibration_servos == false)
      {
        Body_moveVectorComponent[0] = recieved_variables[0].substring(0).toFloat() / 256;
        Body_moveVectorComponent[1] = recieved_variables[1].substring(0).toFloat() / 256;
        Body_moveVectorComponent[2] = recieved_variables[2].substring(0).toFloat() / 400;
        body_height = recieved_variables[3].substring(0).toFloat() / 13;
        height_liftArms = body_height + 10;
        for (size_t i = 0; i < 6; i++)
        {
          Arm[i][2] = body_height;
        }
      }

      if (recieved_variables[4].substring(0).toInt() != (int)calibration_servos)
      {
        if (recieved_variables[4].substring(0).toInt() == 1)   //switch calibration state
        {
          calibration_servos = true;
          //set Arms to calibration position
          for (size_t i = 0; i < 6; i++)
          {
            Arm[i][6] = 0;  //alpha
            Arm[i][7] = 0;  //beta
            Arm[i][8] = 0;  //gamma
          }
          updateServos();
        }
        else
        {
          calibration_servos = false;
          //set Arms to default position
          for (size_t i = 0; i < 6; i++)
          {
            Arm[i][3] = Arm_c[i][3];
            Arm[i][4] = Arm_c[i][4];
            Arm[i][5] = 0;
          }
        }
      }

      recieved_string = "";  //reset string
    }
  }
}
