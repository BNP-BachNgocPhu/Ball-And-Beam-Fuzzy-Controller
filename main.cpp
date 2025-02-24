#include <Arduino.h>
#include <Servo.h>
#include <SimpleKalmanFilter.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16,2);

//#include <TimerOne.h>
// put function declarations here:
#define LED PC13
#define pinservo PA0
#define TRIG_PIN PB12 // CURRENT DISTANCE MEASUREMENT
#define ECHO_PIN PB13//

#define TRIG_PIN2 PA4// SETPOINT DISTANCE MEASUREMENT
#define ECHO_PIN2 PA5
#define trig_set PB2
#define echo_set PB3

Servo Myservo;
SimpleKalmanFilter filter(2, 2, 0.85); // Filter noise from ultrasonic sensor
float e[5];
float edot[5];
float ce[5] = {-0.75, -0.25, 0, 0.25, 0.75}; // X-axis values of error language e
float cedot[5] = {-0.75, -0.42, 0, 0.42, 0.75}; // X-axis values of error rate language edot

float y_out[7] = {-1, -0.8, -0.4, 0, 0.4, 0.8, 1}; // X-axis values of output language (servo angle)
float y_temp[5][5];
float y_star;
//float y_kalman;
float beta[5][5];
int count=0;

// Setup input gain coefficients
float Ke =1/50.000;
float Kedot = 1/90.000;
float Ku = 70; // Degrees

float s_pre;
int s_setpoint;
float s_past=0;
float error;
float errordot;
float time_sample= 0.05; // Sampling time (seconds)
float angle_offset;
float angle_temp;
unsigned long t1;

float trapezoidal_function(float L, float C1, float C2, float R);

void center();
void timerover();

float GetDistance2()
{
  float duration, distanceCm;
   
  digitalWrite(TRIG_PIN2, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN2, LOW);
  
  duration = pulseIn(ECHO_PIN2, HIGH);
  // Convert to distance
  distanceCm = (duration / 29.1 / 2.0);
  
  return distanceCm;
}

float GetDistance()
{
  float duration, distanceCm;
   
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  duration = pulseIn(ECHO_PIN, HIGH);
  // Convert to distance
  distanceCm = (duration / 29.1 / 2.0);
  if(distanceCm <12)
  {
    distanceCm -=1;
  }
  return distanceCm;
}

void setup() {
  // Setup code, runs once:
  Serial.begin(9600);

  lcd.init();                    
  lcd.backlight();
  lcd.setCursor(2,0);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(trig_set, OUTPUT);
  pinMode(echo_set, INPUT);
  pinMode(TRIG_PIN2, OUTPUT);
  pinMode(ECHO_PIN2, INPUT);

  Myservo.attach(pinservo);
  Myservo.write(90);
}

void loop() {
  
  s_pre = filter.updateEstimate(GetDistance()); // Measure current distance
  s_setpoint =(GetDistance2()+1); // Measure setpoint distance
  error = (float)(s_setpoint - s_pre)*Ke; // Calculate input error e normalized to [-1;1]

  // Limit error within [-1;1]
  if(error >1)
  {
    error = 1;
  }else if(error<-1)
  {
    error = -1;
  }
 
  errordot = (float)(((s_setpoint - s_pre)-(s_setpoint - s_past))/time_sample)*Kedot;
  s_past = s_pre;

  // Limit error rate within [-1;1]
  if(errordot >1)
  {
    errordot = 1;
  }else if(errordot<-1)
  {
    errordot = -1;
  }
  
  center(); // Compute output y_star based on e and edot
  // Fuzzy inference method: max-prod, weighted average, Sugeno model

  angle_temp = y_star*Ku; // Compute servo rotation angle
  
  // Limit servo rotation angle
  if(angle_offset > 0)
  {
    angle_offset = -angle_temp +85;
  }
  else 
  {
    angle_offset= 85 + angle_temp ;
  }

  if(angle_offset >120)
  {
    angle_offset = 120;
  }else if(angle_offset<50)
  {
    angle_offset = 50;
  }

  lcd.setCursor(0,0);
  lcd.print("d_out:    ");
  lcd.setCursor(10,0);
  lcd.print(s_pre);

  lcd.setCursor(0,1);
  lcd.print("setpoint: ");
  lcd.setCursor(11,1);

  lcd.print("    ");
  lcd.setCursor(10,1);
  lcd.print(s_setpoint);

  Myservo.write(angle_offset); // Send position to servo
  delay(3);
}

// Trapezoidal function for fuzzy logic
float trapezoidal_function(float x, float L, float C1, float C2, float R)
{
  float val;
  if((x<L))
  {
    val= 0;
  }
  else if(x<C1)
  {
    val = (x-L)/(C1-L);
  }
  else if(x<C2)
  {
    val = 1;
  }
  else if(x<R)
  {
    val = (R-x)/(R-C2);
  }
  else
  {
    val =0;
  }
  return val;
}

void center()
{
  // Error processing
  e[0] = trapezoidal_function(error, -3, -2 , ce[0], ce[1]);   // NB
  e[1] = trapezoidal_function(error, ce[0], ce[1] , ce[1], ce[2]);  //NS
  e[2] = trapezoidal_function(error, ce[1], ce[2] , ce[2], ce[3]);      //ZE
  e[3] = trapezoidal_function(error, ce[2], ce[3] , ce[3], ce[4]);     //PS
  e[4] = trapezoidal_function(error, ce[3], ce[4] , 2 , 3);      //PB

  // Error rate processing
  edot[0] = trapezoidal_function(errordot, -3, -2 , cedot[0], cedot[1]);   // NB
  edot[1] = trapezoidal_function(errordot, cedot[0], cedot[1] , cedot[1], cedot[2]);  //NS
  edot[2] = trapezoidal_function(errordot, cedot[1], cedot[2] , cedot[2], cedot[3]);      //ZE
  edot[3] = trapezoidal_function(errordot, cedot[2], cedot[3] , cedot[3], cedot[4]);     //PS
  edot[4] = trapezoidal_function(errordot, cedot[3], cedot[4] , 2 , 3);      //PB
}

  // Output calculation based on error and error rate
  for (int i=0; i<5; i++)  // i row
  {
    for (int j=0;j<5; j++) // j column
    {
        beta[i][j] = e[i]*edot[j];

        // Calculate y_tam
        if(((i==0)&&(j==0))||((i==0)&&(j==1))||((i==1)&&(j==0)))
        {
          y_tam[i][j] = y_out[6]; // Position PB
        }
        else if(((i==2)&&(j==0))||((i==1)&&(j==1))||((i==0)&&(j==2)))
        {
          y_tam[i][j] = y_out[5]; // Position PM
        }
        else if(((i==3)&&(j==0))||((i==2)&&(j==1))||((i==1)&&(j==2))||((i==0)&&(j==3)))
        {
          y_tam[i][j] = y_out[4]; // Position PS
        }
        else if(((i==4)&&(j==0))||((i==3)&&(j==1))||((i==2)&&(j==2))||((i==1)&&(j==3))||((i==0)&&(j==4)))
        {
          y_tam[i][j] = y_out[3]; // Position ZE
        }
        else if(((i==4)&&(j==1))||((i==3)&&(j==2))||((i==2)&&(j==3))||((i==1)&&(j==4)))
        {
          y_tam[i][j] = y_out[2]; // Position NS
        }
        else if(((i==4)&&(j==2))||((i==3)&&(j==3))||((i==2)&&(j==4)))
        {
          y_tam[i][j] = y_out[1]; // Position NM
        }
        else
        {
          y_tam[i][j] = y_out[0]; // Position NB
        }
    }
  }
  float numerator = 0.0;
  float denominator = 0.0;

  for (int i =0; i<5; i++)
  {
    for(int j=0; j<5; j++)
    {
      numerator = numerator + (beta[i][j])*(y_tam[i][j]);
      denominator = denominator + beta[i][j];
    }
  }
  y_sao = numerator/denominator;
