#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include "PCA9685.h"

bool calibration_servos = true;
float Body_moveVector[3] = {0, 0, 0};             //x, y, angle  in mm/deg   distance/angle per calculated step
float body_height = 60;                           //height from floor to corepoint
float height_liftArms = 80;                       //max height of arms in lift mode
float liftMode_speedMultiplyer = 1;             //higher number will make arms travel faster in lift mode

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
float Arm[6][13] = {
  { 45,  32, body_height,  45,  58, body_height, 0, 0, 0, 0, 0, 0, 0},
  {  0,  32, body_height,   0,  58, body_height, 0, 0, 0, 0, 0, 0, 0},
  {-45,  32, body_height, -45,  58, body_height, 0, 0, 0, 0, 0, 0, 0},
  { 45, -32, body_height,  45, -58, body_height, 0, 0, 0, 0, 0, 0, 0},
  {  0, -32, body_height,   0, -58, body_height, 0, 0, 0, 0, 0, 0, 0},
  {-45, -32, body_height, -45, -58, body_height, 0, 0, 0, 0, 0, 0, 0}, 
};

// all constants
float Arm_c[6][11] = {                            
  {100, 100, 48,  132,  105, -85,   1, -1, -1,  1, -1},
  {100, 100, 48,    0,  146,  -7, -10,  8, -1,  1, -1},
  {100, 100, 48, -132,  105,   0,   5,  3, -1,  1, -1},
  {100, 100, 48,  132, -105,   0,  10,  0, -1, -1,  1},
  {100, 100, 48,    0, -146,   0,   3, -3, -1, -1,  1},
  {100, 100, 48, -132, -105,   6, -90, 83, -1, -1,  1}, 
};

bool Arm_lift[6] = {1, 0, 1, 0, 1, 0};


void getMove();
void getAngles();
void moveServos();
void getmovementinfo();

void setup()
{
  // put your setup code here, to run once:
  Wire.begin();
  Wire.setClock(400000);
  pwm_Board.resetDevices();
  pwm_Board.init(B000000);
  pwm_Board.setPWMFrequency(50);

  servo_Arm6_beta.attach(10);
  servo_Arm6_gamma.attach(11);

  Serial.begin(115200);
  Serial.setTimeout(1);
  Serial.println("Reset");

  getAngles();
  moveServos();
}

void loop()
{
  // put your main code here, to run repeatedly:
  
  // delay(500);

  getmovementinfo();
  // Serial.println("loop");

  if (calibration_servos == false)
  {
    getMove();
    getAngles();
  }
  moveServos();
  
}


void getMove()
{
  float d_Max_liftspeed_A1A3A5 = 0;
  float d_Max_liftspeed_A2A4A6 = 0;
  float r_Body_moveVector = sqrt((Body_moveVector[0] * Body_moveVector[0]) + (Body_moveVector[1] * Body_moveVector[1]));

  for (size_t i = 0; i < 6; i++)
  {
    if (Arm_lift[i] == false)
    {
      //arms on floor
      //moves according to Body_moveVector
      Arm[i][3] = Arm[i][3] - Body_moveVector[0];        //move x of P2
      Arm[i][4] = Arm[i][4] - Body_moveVector[1];        //move y of P2
      if (Body_moveVector[2] != 0)
      {
        float r_temp = sqrt((Arm[i][3] * Arm[i][3]) + (Arm[i][4] * Arm[i][4])); //rotate 
        float angle_temp = acos(Arm[i][3] / r_temp);
        if (i < 3)
        {
          Arm[i][3] = cos(angle_temp - Body_moveVector[2]) * r_temp;
          Arm[i][4] = sin(angle_temp - Body_moveVector[2]) * r_temp;
        }
        else
        {
          Arm[i][3] = cos(angle_temp + Body_moveVector[2]) * r_temp;
          Arm[i][4] = sin(angle_temp + Body_moveVector[2]) * r_temp;
        }
      }
      // d
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

  if ((Arm_lift[0] == false) & (Arm_lift[2] == false) & (Arm_lift[4] == false))
  {
    if (Arm[0][11] < Arm[2][11])
    {
      if (Arm[2][11] < Arm[4][11])
      {
        d_Max_liftspeed_A1A3A5 = Arm[4][11];
      }
      else
      {
        d_Max_liftspeed_A1A3A5 = Arm[2][11];
      }
    }
    else
    {
      if (Arm[0][11] < Arm[4][11])
      {
        d_Max_liftspeed_A1A3A5 = Arm[4][11];
      }
      else
      {
        d_Max_liftspeed_A1A3A5 = Arm[0][11];
      }
    }
  }
  if ((Arm_lift[1] == false) & (Arm_lift[3] == false) & (Arm_lift[5] == false))
  {
    if (Arm[1][11] < Arm[3][11])
    {
      if (Arm[3][11] < Arm[5][11])
      {
        d_Max_liftspeed_A2A4A6 = Arm[5][11];
      }
      else
      {
        d_Max_liftspeed_A2A4A6 = Arm[3][11];
      }
    }
    else
    {
      if (Arm[1][11] < Arm[5][11])
      {
        d_Max_liftspeed_A2A4A6 = Arm[5][11];
      }
      else
      {
        d_Max_liftspeed_A2A4A6 = Arm[1][11];
      }
    }
  }

  for (size_t i = 0; i < 6; i++)
  {
    if (Arm_lift[i] == true)
    {
      //arm in lift mode -- moves in direction of target point at which the arm touches floor again
      float P_target[2] = {0, 0};

      if (r_Body_moveVector != 0) // target point x/y in mm
      {
        P_target[0] = Arm_c[i][3] + ((Arm_c[i][2] * Body_moveVector[0]) / r_Body_moveVector);
        P_target[1] = Arm_c[i][4] + ((Arm_c[i][2] * Body_moveVector[1]) / r_Body_moveVector);
      }
      else
      {
        P_target[0] = Arm_c[i][3];
        P_target[0] = Arm_c[i][4];
      }
      float V_P2Pt[2] = {P_target[0] - Arm[i][3], P_target[1] - Arm[i][4]}; //  x/y
      if ((V_P2Pt[0] != 0) | (V_P2Pt[1] != 0))
      {
        float liftspeed_temp;
        if ((i == 0) | (i == 2) | (i == 4))
        {
          if (d_Max_liftspeed_A1A3A5 == 0)
          {
            liftspeed_temp = r_Body_moveVector;
          }
          else
          {
            liftspeed_temp = ((r_Body_moveVector * sqrt((V_P2Pt[0] * V_P2Pt[0]) + (V_P2Pt[1] * V_P2Pt[1]))) / d_Max_liftspeed_A1A3A5);
          }
        }
        else
        {
          if (d_Max_liftspeed_A2A4A6 == 0)
          {
            liftspeed_temp = r_Body_moveVector;
          }
          else
          {
            liftspeed_temp = ((r_Body_moveVector * sqrt((V_P2Pt[0] * V_P2Pt[0]) + (V_P2Pt[1] * V_P2Pt[1]))) / d_Max_liftspeed_A2A4A6);
          }

        }
        Arm[i][3] = Arm[i][3] + ((liftspeed_temp * V_P2Pt[0]) / (sqrt((V_P2Pt[0] * V_P2Pt[0]) + (V_P2Pt[1] * V_P2Pt[1]))));
        Arm[i][4] = Arm[i][4] + ((liftspeed_temp * V_P2Pt[1]) / (sqrt((V_P2Pt[0] * V_P2Pt[0]) + (V_P2Pt[1] * V_P2Pt[1]))));
        Arm[i][5] = (height_liftArms * (Arm_c[i][2] - sqrt(((Arm_c[i][3] - Arm[i][3]) * (Arm_c[i][3] - Arm[i][3])) + ((Arm_c[i][4] - Arm[i][4]) * (Arm_c[i][4] - Arm[i][4]))))) / Arm_c[i][2];
      }
      if (Body_moveVector[2] != 0)
      {
        float r_temp = sqrt((Arm[i][3] * Arm[i][3]) + (Arm[i][4] * Arm[i][4])); //rotate 
        float angle_temp = acos(Arm[i][3] / r_temp);
        if (i < 3)
        {
          Arm[i][3] = cos(angle_temp + Body_moveVector[2]) * r_temp;
          Arm[i][4] = sin(angle_temp + Body_moveVector[2]) * r_temp;
        }
        else
        {
          Arm[i][3] = cos(angle_temp - Body_moveVector[2]) * r_temp;
          Arm[i][4] = sin(angle_temp - Body_moveVector[2]) * r_temp;
        }
    }
  }

  for (size_t i = 0; i < 6; i++)
  {
    if (Arm_lift[i] == true)
    {
      if (Arm[i][5] <= 0)
      {
        Arm_lift[i] = false;
        Arm[i][5] = 0;
        Arm[i][3] = Arm[i][9];
        Arm[i][4] = Arm[i][10];
      }
    }
    else
    {
      if (sqrt(((Arm_c[i][3] - Arm[i][3]) * (Arm_c[i][3] - Arm[i][3])) + ((Arm_c[i][4] - Arm[i][4]) * (Arm_c[i][4] - Arm[i][4]))) > Arm_c[i][2])
      {
        Arm_lift[0] = true;
        Arm_lift[2] = true;
        Arm_lift[4] = true;
        Arm[0][3] = Arm[0][9];
        Arm[0][4] = Arm[0][10];
        Arm[2][3] = Arm[2][9];
        Arm[2][4] = Arm[2][10];
        Arm[4][3] = Arm[4][9];
        Arm[4][4] = Arm[4][10];
      }
    }
  }
}
}
void getAngles()
{
  for (size_t i = 0; i < 6; i++)
  {
    float lpp = sqrt((Arm[i][0] * Arm[i][0]) + (Arm[i][1] * Arm[i][1]) + (Arm[i][2] * Arm[i][2]));
    Arm[i][6] = (atan(Arm[i][0] / Arm[i][1]) * 180) / PI;
    float epsilon = (asin(Arm[i][2] / lpp) * 180) / PI;
    float delta = acos(((Arm_c[i][0] * Arm_c[i][0]) + (lpp * lpp) - (Arm_c[i][1] * Arm_c[i][1])) / (2 * Arm_c[i][0] * lpp));
    Arm[i][8] = ((acos(((Arm_c[i][0] * Arm_c[i][0]) + (Arm_c[i][1] * Arm_c[i][1]) - (lpp * lpp)) / (2 * Arm_c[i][0] * Arm_c[i][1])) * 180) / PI) - 90;
    Arm[i][7] = 90 - (((delta * 180) / PI) - epsilon);
  }
  
}
void moveServos()
{
  pwm_Board.setChannelPWM(0,  servo_Arm1_alpha.pwmForAngle((Arm[0][6] + Arm_c[0][5]) * Arm_c[0][8]));
  pwm_Board.setChannelPWM(1,   servo_Arm1_beta.pwmForAngle((Arm[0][7] + Arm_c[0][6]) * Arm_c[0][9]));
  pwm_Board.setChannelPWM(2,  servo_Arm1_gamma.pwmForAngle((Arm[0][8] + Arm_c[0][7]) * Arm_c[0][10]));
 
  pwm_Board.setChannelPWM(6,  servo_Arm2_alpha.pwmForAngle((Arm[1][6] + Arm_c[1][5]) * Arm_c[1][8]));
  pwm_Board.setChannelPWM(7,   servo_Arm2_beta.pwmForAngle((Arm[1][7] + Arm_c[1][6]) * Arm_c[1][9]));
  pwm_Board.setChannelPWM(8,  servo_Arm2_gamma.pwmForAngle((Arm[1][8] + Arm_c[1][7]) * Arm_c[1][10]));
 
  pwm_Board.setChannelPWM(12, servo_Arm3_alpha.pwmForAngle((Arm[2][6] + Arm_c[2][5]) * Arm_c[2][8]));
  pwm_Board.setChannelPWM(13,  servo_Arm3_beta.pwmForAngle((Arm[2][7] + Arm_c[2][6]) * Arm_c[2][9]));
  pwm_Board.setChannelPWM(14, servo_Arm3_gamma.pwmForAngle((Arm[2][8] + Arm_c[2][7]) * Arm_c[2][10]));
 
  pwm_Board.setChannelPWM(3,  servo_Arm4_alpha.pwmForAngle((Arm[3][6] + Arm_c[3][5]) * Arm_c[3][8]));
  pwm_Board.setChannelPWM(4,   servo_Arm4_beta.pwmForAngle((Arm[3][7] + Arm_c[3][6]) * Arm_c[3][9]));
  pwm_Board.setChannelPWM(5,  servo_Arm4_gamma.pwmForAngle((Arm[3][8] + Arm_c[3][7]) * Arm_c[3][10]));
 
  pwm_Board.setChannelPWM(9,  servo_Arm5_alpha.pwmForAngle((Arm[4][6] + Arm_c[4][5]) * Arm_c[4][8]));
  pwm_Board.setChannelPWM(10,  servo_Arm5_beta.pwmForAngle((Arm[4][7] + Arm_c[4][6]) * Arm_c[4][9]));
  pwm_Board.setChannelPWM(11, servo_Arm5_gamma.pwmForAngle((Arm[4][8] + Arm_c[4][7]) * Arm_c[4][10]));

  pwm_Board.setChannelPWM(15, servo_Arm6_alpha.pwmForAngle((Arm[5][6] + Arm_c[5][5]) * Arm_c[5][8]));
                                     servo_Arm6_beta.write((Arm[5][7] + Arm_c[5][6]) * Arm_c[5][9]);
                                    servo_Arm6_gamma.write((Arm[5][8] + Arm_c[5][7]) * Arm_c[5][10]);
}
void getmovementinfo()
{
  String serialIn = Serial.readString();

  if (serialIn[0] == 'M')
  {
    if (serialIn[2] == 'V')
    {
      if (serialIn[3] == 'x')
      {
        Body_moveVector[0] = serialIn.substring(4).toFloat();
        Serial.print("Vx changed: ");
        Serial.println(Body_moveVector[0]);
      }
      if (serialIn[3] == 'y')
      {
        Body_moveVector[1] = serialIn.substring(4).toFloat();
        Serial.print("Vy changed: ");
        Serial.println(Body_moveVector[1]);
      }
      if (serialIn[3] == 'a')
      {
        Body_moveVector[2] = serialIn.substring(4).toFloat();
        Serial.print("Va changed: ");
        Serial.println(Body_moveVector[2]);
      }
    }
    
  }

  if (serialIn[0] == 'C')
  {
    if (serialIn[2] == 't')
    {
      calibration_servos = true;
      Serial.println("Calibration_servos changed: true");
      Arm[0][0] = 45;
      Arm[0][1] = 58;
      Arm[0][2] = body_height;
      Arm[1][0] = 0;
      Arm[1][1] = 58;
      Arm[1][2] = body_height;
      Arm[2][0] = -45;
      Arm[2][1] = 58;
      Arm[2][2] = body_height;
      Arm[3][0] = 45;
      Arm[3][1] = -58;
      Arm[3][2] = body_height;
      Arm[4][0] = 0;
      Arm[4][1] = -58;
      Arm[4][2] = body_height;
      Arm[5][0] = -45;
      Arm[5][1] = -58;
      Arm[5][2] = body_height;
      getAngles();
      moveServos();
    }
    if (serialIn[2] == 'f')
    {
      calibration_servos = false;
      Serial.println("Calibration_servos changed: false");

      Arm[0][0] = 132;
      Arm[0][1] = 105;
      Arm[0][2] = 0;
      Arm[1][0] = 0;
      Arm[1][1] = 146;
      Arm[1][2] = 0;
      Arm[2][0] = -132;
      Arm[2][1] = 105;
      Arm[2][2] = 0;
      Arm[3][0] = 132;
      Arm[3][1] = -105;
      Arm[3][2] = 0;
      Arm[4][0] = 0;
      Arm[4][1] = -146;
      Arm[4][2] = 0;
      Arm[5][0] = -132;
      Arm[5][1] = -105;
      Arm[5][2] = 0;
    }
    // if (serialIn.substring(2, 7) == "S_off")
    // {
    //   if (serialIn.substring(8, 10) == "A1")
    //   {
    //     if (serialIn[11] == 'a') {Servo_Arm1_alpha_offset = serialIn.substring(12).toFloat();
    //     Serial.print("Calibration_servo_offset: ");
    //     Serial.println(Servo_Arm1_alpha_offset);}
    //     if (serialIn[11] == 'b') {Servo_Arm1_beta_offset = serialIn.substring(12).toFloat();
    //     Serial.print("Calibration_servo_offset: ");
    //     Serial.println(Servo_Arm1_beta_offset);}
    //     if (serialIn[11] == 'g') {Servo_Arm1_gamma_offset = serialIn.substring(12).toFloat();
    //     Serial.print("Calibration_servo_offset: ");
    //     Serial.println(Servo_Arm1_gamma_offset);}
    //   }
    //   if (serialIn.substring(8, 10) == "A2")
    //   {
    //     if (serialIn[11] == 'a') {Servo_Arm2_alpha_offset = serialIn.substring(12).toFloat();
    //     Serial.print("Calibration_servo_offset: ");
    //     Serial.println(Servo_Arm2_alpha_offset);}
    //     if (serialIn[11] == 'b') {Servo_Arm2_beta_offset = serialIn.substring(12).toFloat();
    //     Serial.print("Calibration_servo_offset: ");
    //     Serial.println(Servo_Arm2_beta_offset);}
    //     if (serialIn[11] == 'g') {Servo_Arm2_gamma_offset = serialIn.substring(12).toFloat();
    //     Serial.print("Calibration_servo_offset: ");
    //     Serial.println(Servo_Arm2_gamma_offset);}
    //   }
    //   if (serialIn.substring(8, 10) == "A3")
    //   {
    //     if (serialIn[11] == 'a') {Servo_Arm3_alpha_offset = serialIn.substring(12).toFloat();
    //     Serial.print("Calibration_servo_offset: ");
    //     Serial.println(Servo_Arm3_alpha_offset);}
    //     if (serialIn[11] == 'b') {Servo_Arm3_beta_offset = serialIn.substring(12).toFloat();
    //     Serial.print("Calibration_servo_offset: ");
    //     Serial.println(Servo_Arm3_beta_offset);}
    //     if (serialIn[11] == 'g') {Servo_Arm3_gamma_offset = serialIn.substring(12).toFloat();
    //     Serial.print("Calibration_servo_offset: ");
    //     Serial.println(Servo_Arm3_gamma_offset);}
    //   }
    //   if (serialIn.substring(8, 10) == "A4")
    //   {
    //     if (serialIn[11] == 'a') {Servo_Arm4_alpha_offset = serialIn.substring(12).toFloat();
    //     Serial.print("Calibration_servo_offset: ");
    //     Serial.println(Servo_Arm4_alpha_offset);}
    //     if (serialIn[11] == 'b') {Servo_Arm4_beta_offset = serialIn.substring(12).toFloat();
    //     Serial.print("Calibration_servo_offset: ");
    //     Serial.println(Servo_Arm4_beta_offset);}
    //     if (serialIn[11] == 'g') {Servo_Arm4_gamma_offset = serialIn.substring(12).toFloat();
    //     Serial.print("Calibration_servo_offset: ");
    //     Serial.println(Servo_Arm4_gamma_offset);}
    //   }
    //   if (serialIn.substring(8, 10) == "A5")
    //   {
    //     if (serialIn[11] == 'a') {Servo_Arm5_alpha_offset = serialIn.substring(12).toFloat();
    //     Serial.print("Calibration_servo_offset: ");
    //     Serial.println(Servo_Arm5_alpha_offset);}
    //     if (serialIn[11] == 'b') {Servo_Arm5_beta_offset = serialIn.substring(12).toFloat();
    //     Serial.print("Calibration_servo_offset: ");
    //     Serial.println(Servo_Arm5_beta_offset);}
    //     if (serialIn[11] == 'g') {Servo_Arm5_gamma_offset = serialIn.substring(12).toFloat();
    //     Serial.print("Calibration_servo_offset: ");
    //     Serial.println(Servo_Arm5_gamma_offset);}
    //   }
    //   if (serialIn.substring(8, 10) == "A6")
    //   {
    //     if (serialIn[11] == 'a') {Servo_Arm6_alpha_offset = serialIn.substring(12).toFloat();
    //     Serial.print("Calibration_servo_offset: ");
    //     Serial.println(Servo_Arm6_alpha_offset);}
    //     if (serialIn[11] == 'b') {Servo_Arm6_beta_offset = serialIn.substring(12).toFloat();
    //     Serial.print("Calibration_servo_offset: ");
    //     Serial.println(Servo_Arm6_beta_offset);}
    //     if (serialIn[11] == 'g') {Servo_Arm6_gamma_offset = serialIn.substring(12).toFloat();
    //     Serial.print("Calibration_servo_offset: ");
    //     Serial.println(Servo_Arm6_gamma_offset);}
    //   }
    // }

    // if (serialIn.substring(2, 8) == "S_flip")
    // {
    //   if (serialIn.substring(9, 11) == "A1")
    //   {
    //     if (serialIn[12] == 'a') {flip_servo_arm1_alpha = serialIn.substring(13).toFloat();
    //     Serial.print("Calibration_servo_flip: ");
    //     Serial.println(flip_servo_arm1_alpha);}
    //     if (serialIn[12] == 'b') {flip_servo_arm1_beta = serialIn.substring(13).toFloat();
    //     Serial.print("Calibration_servo_flip: ");
    //     Serial.println(flip_servo_arm1_beta);}
    //     if (serialIn[12] == 'g') {flip_servo_arm1_gamma = serialIn.substring(13).toFloat();
    //     Serial.print("Calibration_servo_flip: ");
    //     Serial.println(flip_servo_arm1_gamma);}
    //   }
    //   if (serialIn.substring(9, 11) == "A2")
    //   {
    //     if (serialIn[12] == 'a') {flip_servo_arm2_alpha = serialIn.substring(13).toFloat();
    //     Serial.print("Calibration_servo_flip: ");
    //     Serial.println(flip_servo_arm2_alpha);}
    //     if (serialIn[12] == 'b') {flip_servo_arm2_beta = serialIn.substring(13).toFloat();
    //     Serial.print("Calibration_servo_flip: ");
    //     Serial.println(flip_servo_arm2_beta);}
    //     if (serialIn[12] == 'g') {flip_servo_arm2_gamma = serialIn.substring(13).toFloat();
    //     Serial.print("Calibration_servo_flip: ");
    //     Serial.println(flip_servo_arm2_gamma);}
    //   }
    //   if (serialIn.substring(9, 11) == "A3")
    //   {
    //     if (serialIn[12] == 'a') {flip_servo_arm3_alpha = serialIn.substring(13).toFloat();
    //     Serial.print("Calibration_servo_flip: ");
    //     Serial.println(flip_servo_arm3_alpha);}
    //     if (serialIn[12] == 'b') {flip_servo_arm3_beta = serialIn.substring(13).toFloat();
    //     Serial.print("Calibration_servo_flip: ");
    //     Serial.println(flip_servo_arm3_beta);}
    //     if (serialIn[12] == 'g') {flip_servo_arm3_gamma = serialIn.substring(13).toFloat();
    //     Serial.print("Calibration_servo_flip: ");
    //     Serial.println(flip_servo_arm3_gamma);}
    //   }
    //   if (serialIn.substring(9, 11) == "A4")
    //   {
    //     if (serialIn[12] == 'a') {flip_servo_arm4_alpha = serialIn.substring(13).toFloat();
    //     Serial.print("Calibration_servo_flip: ");
    //     Serial.println(flip_servo_arm4_alpha);}
    //     if (serialIn[12] == 'b') {flip_servo_arm4_beta = serialIn.substring(13).toFloat();
    //     Serial.print("Calibration_servo_flip: ");
    //     Serial.println(flip_servo_arm4_beta);}
    //     if (serialIn[12] == 'g') {flip_servo_arm4_gamma = serialIn.substring(13).toFloat();
    //     Serial.print("Calibration_servo_flip: ");
    //     Serial.println(flip_servo_arm4_gamma);}
    //   }
    //   if (serialIn.substring(9, 11) == "A5")
    //   {
    //     if (serialIn[12] == 'a') {flip_servo_arm5_alpha = serialIn.substring(13).toFloat();
    //     Serial.print("Calibration_servo_flip: ");
    //     Serial.println(flip_servo_arm5_alpha);}
    //     if (serialIn[12] == 'b') {flip_servo_arm5_beta = serialIn.substring(13).toFloat();
    //     Serial.print("Calibration_servo_flip: ");
    //     Serial.println(flip_servo_arm5_beta);}
    //     if (serialIn[12] == 'g') {flip_servo_arm5_gamma = serialIn.substring(13).toFloat();
    //     Serial.print("Calibration_servo_flip: ");
    //     Serial.println(flip_servo_arm5_gamma);}
    //   }
    //   if (serialIn.substring(9, 11) == "A6")
    //   {
    //     if (serialIn[12] == 'a') {flip_servo_arm6_alpha = serialIn.substring(13).toFloat();
    //     Serial.print("Calibration_servo_flip: ");
    //     Serial.println(flip_servo_arm6_alpha);}
    //     if (serialIn[12] == 'b') {flip_servo_arm6_beta = serialIn.substring(13).toFloat();
    //     Serial.print("Calibration_servo_flip: ");
    //     Serial.println(flip_servo_arm6_beta);}
    //     if (serialIn[12] == 'g') {flip_servo_arm6_gamma = serialIn.substring(13).toFloat();
    //     Serial.print("Calibration_servo_flip: ");
    //     Serial.println(flip_servo_arm6_gamma);}
    //   }
    // } 
  }
}
