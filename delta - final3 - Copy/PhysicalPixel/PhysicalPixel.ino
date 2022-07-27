#include <LiquidCrystal.h>
#include <Servo.h>
Servo myservo1;
Servo myservo2;
Servo myservo3;
LiquidCrystal lcd(12, 11, 5,4,3,2);

const float e = 115.0; // chiều dài cạnh tam giác để bàn làm việc (mặt phẳng dưới)
const float f = 457.3; // chiều dài cạnh tam giác để bàn gắn động cơ (mặt phẳng trên)
const float re = 232.0; // chiều dài khớp hình bình hành
const float rf = 112.0; // chiều dài khớp gắn động cơ

// hằng số lượng giác
const float sqrt3 = sqrt(3.0);
const float pi = 3.141592653;
const float sin120 = sqrt3/2.0;
const float cos120 = -0.5;
const float tan60 = sqrt3;
const float sin30 = 0.5;
const float tan30 = 1/sqrt3;

String string;
String recvString;
char *strings[5]; // an array of pointers to the pieces of the above array after strtok()
char *ptr = NULL;
int gripper;
float x0;
float y0;
float z0;
float theta1, theta2, theta3;

void setup() 
{
  myservo1.attach(9);
  myservo2.attach(8);
  myservo3.attach(7);
  Serial.begin(57600);
  lcd.begin(16, 2);
  lcd.print("trang thai!");
}
  // động học nghịch tính góc servo từ tọa độ (x0, y0, z0)
//
int delta_calcAngleYZ(float x0, float y0, float z0, float &theta) {
  float y1 = -0.5*0.57735*e;
  y0 -= 0.5*0.57735*f;
  // z = a+ b*y
  float a = (x0*x0 + y0*y0 + z0*z0 + rf*rf -re*re - y1*y1)/(2*z0);
  float b = (y1-y0)/z0;
  // discriminant
  float d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf);
  if (d<0) return -1; // non-existing point
  float yj = (y1 - a*b - sqrt(d))/(b*b + 1);
  float zj = a + b*yj;
  theta = 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);
  return 0;
  }
    // inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
    // returned status: 0=Ok, -1 = non-existing position
    
int delta_calcInverse(float x0, float y0, float z0, float &theta1, float &theta2, float &theta3) {
  theta1 = theta2 = theta3 = 0;
  int status = delta_calcAngleYZ(x0, y0, z0, theta1);
  if (status == 0) status = delta_calcAngleYZ(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0, theta2);
  if (status == 0) status = delta_calcAngleYZ(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0, theta3);
  return status;
  }  
void loop()
{
  while (Serial.available())
  {   
    string = Serial.readStringUntil('\n');
    char recvString[50];
    string.toCharArray(recvString, 50);
    byte index = 0;
    ptr = strtok(recvString, ",");
    while (ptr != NULL)
    {
      strings[index] = ptr;
      index++;
      ptr = strtok(NULL, ",");
      }
    gripper = atoi(strings[0]);
    x0 = atof(strings[1]);
    y0 = atof(strings[2]);
    z0 = atof(strings[3]);
    
    lcd.clear();
    lcd.setCursor(0,1);
    delta_calcInverse(x0, y0, z0, theta1, theta2, theta3);
    lcd.print(gripper);
    lcd.print(" ");
    lcd.print(theta1);
    myservo1.write(theta1);
    myservo2.write(theta2);
    myservo3.write(theta3);
    delay(15); 
  }
}
