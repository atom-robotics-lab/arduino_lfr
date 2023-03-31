#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// IR Sensors
int sensor1 = A0;      // Left most sensor
int sensor2 = A1;
int sensor3 = A2;
int sensor4 = A3;
int sensor5 = A6;      // Right most sensor

int ob_sensor = A7;

// Initial Values of Sensors
int sensor[5] = {0, 0, 0, 0, 0};

// Motor Variables
int motorInput1 = 11;
int motorInput2 = 10;
int motorInput3 = 3;
int motorInput4 = 5;

int obstacle = 0;

int left_stop=100;

int nsensor1 = 2;
int nsensor2 = 4;
int nsensor3 = 6;
int nsensor4 = 7;

int node=0;
int node_value;

int nd_sensor[4]={0,0,0,0};

//Initial Speed of Motor
int intial_motor_speed = 150;
int left_motor_speed = 150;
int right_motor_speed = 150;

// PID Constants
float Kp = 40;
float Ki = 0;
float Kd = 0;


float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;

void setup()
{
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
 Serial.println(("SSD1306 allocation failed"));
}
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);

  pinMode(motorInput1, OUTPUT);
  pinMode(motorInput2, OUTPUT);
  pinMode(motorInput3, OUTPUT);
  pinMode(motorInput4, OUTPUT);
  Serial.begin(9600);
}

void loop(){ 
      Serial.println(2);

  read_sensor_values();

  // Node detection code
  
//  if(obstacle==1){
//  reverse();
//  delay(50);
//  do{
//    read_sensor_values();
//    sharpLeftTurn();
//   }while(error!=1 || error!=0 || error!=-1);
// }


//  if (sensor[2]==0){
    //if(node_detection()){
//  if(node==0){
//    intial_motor_speed=50;
//    Kp=13;
//    }  
//  else if(node==1){
//    intial_motor_speed=150;
//    Kp=40;
//    }
//  else if (node=2){
//    read_sensor_value();
//    while(error==102){
//      left_motor_speed=100;
//      right_motor_speed=100;
//      forward();
//      }
//      do{
//        read_sensor_value();
//        left_motor_speed()
//        }while(error!=1 || error!=0 || error!=-1)
//    left_stop=0;
//    }
//  }  

 //   }

 
  if (error == 100) {
    if(left_stop>6){
    Serial.print("\t");
    Serial.println("Left");
    left_motor_speed = 100;
    right_motor_speed = 100;
    forward();
    delay(300);
    stop_bot();// Make left turn untill it detects straight path
    delay(500);
    sharpLeftTurn();
    delay(200);
    read_sensor_values();
    do {
      read_sensor_values();
      sharpLeftTurn();
    } while (error != 0);
    }
//  else{
//    forward();
//    delay(100);
//    left_stop++;
//    } 
  }
  else if (error == 101) {          // Make right turn in case of it detects only right path (it will go into forward direction in case of staright and right "|--")
    // untill it detects straight path.
    Serial.print("\t");
    Serial.println("Right");
    left_motor_speed = 100;
    right_motor_speed = 100;
    forward();
    delay(300);
    stop_bot();// Make left turn untill it detects straight path
    delay(500);
    read_sensor_values();
    while (error != 0) {
      read_sensor_values();
      sharpRightTurn();
    };
  }
  else if (error == 102) {        // Make left turn untill it detects straight path  


//else{
    Serial.print("\t");
    Serial.print(left_motor_speed);
    Serial.print("\t");
    Serial.println(right_motor_speed);
    forward();
    //delay(200);
    //read_sensor_values();
    //while(error!=0){
    // sharpLeftTurn();
    //  read_sensor_values();
    //  }
//  }
}
else if (error==103){
  left_motor_speed=100;
  right_motor_speed=100;
  forward();
  delay(300);
  if (error==103){
    stop_bot();
    }
  else{
    stop_bot();// Make left turn untill it detects straight path
    delay(500);
    sharpLeftTurn();
    delay(200);
    read_sensor_values();
    do {
      read_sensor_values();
      sharpLeftTurn();
    } while (error != 0);
    }
  }

  else {
    calculate_pid();
    motor_control();
  }
}

boolean node_detection(){
// boolean flag=true;
  //  Serial.print(nd_sensor[0]);
//  Serial.print("\t");
//  Serial.print(nd_sensor[1]);
//  Serial.print("\t");
//  Serial.print(nd_sensor[2]);
//  Serial.print("\t");
//  Serial.println(nd_sensor[3]);
//      if ((nd_sensor[0]==1) && (nd_sensor[1]==1) && (nd_sensor[2]==1) && (nd_sensor[3]==0)){
//        Serial.print("1110 \t");
//        node_value=14;
//        Serial.println(node_value);
          display.clearDisplay();
          display.setTextSize(2); // Draw 2X-scale text
          display.setTextColor(WHITE);
          display.setCursor(0, 0);
          display.println((node_value));
          display.display();      // Show initial text
          
//        node++;
//        }
//      else if ((nd_sensor[0]==1) && (nd_sensor[1]==1) && (nd_sensor[2]==0) && (nd_sensor[3]==1)){
//        Serial.print("1101");
//        node_value=13;
//        Serial.println(node_value);
          display.clearDisplay();
          display.setTextSize(2); // Draw 2X-scale text
          display.setTextColor(WHITE);
          display.setCursor(0, 0);
          display.println((node_value));
          display.display(); 
//        node++;
//        }
//      else if ((nd_sensor[0]==0) && (nd_sensor[0]==1) && (nd_sensor[2]==1) && (nd_sensor[3]==0)){
//        Serial.print("0110");
//        node_value=6;
//        Serial.println(node_value);
          display.clearDisplay();
          display.setTextSize(2); // Draw 2X-scale text
          display.setTextColor(WHITE);
          display.setCursor(0, 0);
          display.println((node_value));
          display.display(); 
//        node++;
//        }
//      else if ((nd_sensor[0]==1) && (nd_sensor[0]==0) && (nd_sensor[2]==1) && (nd_sensor[3]==0)){
//        Serial.print("1010");
//        node_value=10;
//        Serial.println(node_value);
          display.clearDisplay();
          display.setTextSize(2); // Draw 2X-scale text
          display.setTextColor(WHITE);
          display.setCursor(0, 0);
          display.println((node_value));
          display.display(); 
//        node++;
//        }
//      else if ((nd_sensor[0]==0) && (nd_sensor[0]==1) && (nd_sensor[2]==0) && (nd_sensor[3]==1)){
//        Serial.print("0101");
//        node_value=5;
//        Serial.println(node_value);
          display.clearDisplay();
          display.setTextSize(2); // Draw 2X-scale text
          display.setTextColor(WHITE);
          display.setCursor(0, 0);
          display.println((node_value));
          display.display(); 
//        node++;
//        }          
//      else if ((nd_sensor[0]==1) && (nd_sensor[0]==0) && (nd_sensor[2]==0) && (nd_sensor[3]==1)){
//        Serial.print("1001");
//        node_value=9;
//        Serial.println(node_value);
          display.clearDisplay();
          display.setTextSize(2); // Draw 2X-scale text
          display.setTextColor(WHITE);
          display.setCursor(0, 0);
          display.println((node_value));
          display.display(); 
//        node++;
//        }          
//      else if((nd_sensor[0]==1) && (nd_sensor[1]==1) && (nd_sensor[2]==0) && (nd_sensor[3]==0)){
//        node_value=12;
//        Serial.println(node_value);
          display.clearDisplay();
          display.setTextSize(2); // Draw 2X-scale text
          display.setTextColor(WHITE);
          display.setCursor(0, 0);
          display.println((node_value));
          display.display(); 
//        node++;
//      }
//      else if((nd_sensor[0]==0) && (nd_sensor[1]==1) && (nd_sensor[2]==0) && (nd_sensor[3]==0)){
//        node_value=8;
//        Serial.println(node_value);
          display.clearDisplay();
          display.setTextSize(2); // Draw 2X-scale text
          display.setTextColor(WHITE);
          display.setCursor(0, 0);
          display.println((node_value));
          display.display(); 
//        node++;
//        }
//      else if((nd_sensor[0]==1) && (nd_sensor[1]==0) && (nd_sensor[2]==0) && (nd_sensor[3]==0)){
//        Serial.println("1000");
//        node_value=8;
//        Serial.println(node_value);
          display.clearDisplay();
          display.setTextSize(2); // Draw 2X-scale text
          display.setTextColor(WHITE);
          display.setCursor(0, 0);
          display.println((node_value));
          display.display(); 
//        node++;
//      else{
//        flag=false;
//}
//return flag;
  }

void read_sensor_values()
{
  sensor[0] = !digitalRead(sensor1);
  sensor[1] = !digitalRead(sensor2);
  sensor[2] = !digitalRead(sensor3);
  sensor[3] = digitalRead(nsensor4);
 // sensor[4] =  analogRead(sensor5) > 300 ? 0 : 1;

  nd_sensor[0] = digitalRead(nsensor1);
  nd_sensor[2] = digitalRead(nsensor2);
  nd_sensor[1] = digitalRead(nsensor3);
  nd_sensor[3] = digitalRead(nsensor4);
//  
  obstacle = analogRead(ob_sensor) > 300 ? 1 : 0 ;
  Serial.print(sensor[0]);
  Serial.print("\t");
  Serial.print(sensor[1]);
  Serial.print("\t");
  Serial.print(sensor[2]);
  Serial.print("\t");
  Serial.print(sensor[3]);
  Serial.print("\t");
  Serial.println(sensor[4]);
  if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0) )
    error = 3;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 0))
    error = 2;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
    error = 1;
  else if ((sensor[0] == 0 ) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))
    error = 0;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0))
    error = -1;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 0))
    error = -2;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1))
    error = -3;
  else if ((sensor[0] == 1) && (sensor[1]==1) && (sensor[2] == 1)) // Turn robot left side
    error = 100;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) // Turn robot right side
    error = 101;
 // else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1)) // finish
 //   error = 103;
}

void calculate_pid()
{
  P = error;
  I = I + previous_I;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_I = I;
  previous_error = error;
}

void motor_control()
{
  // Calculating the effective motor speed:
  left_motor_speed = intial_motor_speed - PID_value;
  right_motor_speed = intial_motor_speed + PID_value;
  // The motor speed should not exceed the max PWM value
  left_motor_speed = constrain(left_motor_speed, 0, 255);
  right_motor_speed = constrain(right_motor_speed, 0, 255);
  //following lines of code are to make the bot move forward
  forward();
}

void forward()
{
  /*The pin numbers and high, low values might be different depending on your connections */

  analogWrite(motorInput1, 0);
  analogWrite(motorInput2,left_motor_speed);
  analogWrite(motorInput3, 0);
  analogWrite(motorInput4, right_motor_speed);
}
void reverse()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  analogWrite(motorInput1, left_motor_speed);
  analogWrite(motorInput2, 0);
  analogWrite(motorInput3, right_motor_speed);
  analogWrite(motorInput4, 0);
}
void right()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  
  analogWrite(motorInput1, 0);
  analogWrite(motorInput2, left_motor_speed);
  analogWrite(motorInput3, 0);
  analogWrite(motorInput4, 0);
}
void left()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  
  analogWrite(motorInput1, 0);
  analogWrite(motorInput2, 0);
  analogWrite(motorInput3, right_motor_speed);
  analogWrite(motorInput4, 0);
}
void sharpRightTurn() {
  /*The pin numbers and high, low values might be different depending on your connections */
  
  digitalWrite(motorInput1, LOW);
  //digitalWrite(motorInput2, HIGH);
  //digitalWrite(motorInput3, HIGH);
  analogWrite(motorInput2, 100);
  analogWrite(motorInput3, 100);
  digitalWrite(motorInput4, LOW);
}
void sharpLeftTurn() {
  /*The pin numbers and high, low values might be different depending on your connections */
  
  //digitalWrite(motorInput1, HIGH);
  analogWrite(motorInput1, 100);
  digitalWrite(motorInput2, LOW);
  digitalWrite(motorInput3, LOW);
  analogWrite(motorInput4, 100);
  //digitalWrite(motorInput4, HIGH);
}
void stop_bot()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  
  digitalWrite(motorInput1, HIGH);
  digitalWrite(motorInput2, HIGH);
  digitalWrite(motorInput3, HIGH);
  digitalWrite(motorInput4, HIGH);
}
