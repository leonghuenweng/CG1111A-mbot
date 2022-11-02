#include "MeMCore.h"

MeLineFollower lineFinder(PORT_1); // linefollower sensor connected to port 1
MeUltrasonicSensor ultraSensor(PORT_2); // ultrasonic sensor connected to port 2
//int status = 1; // global status; 0 = do nothing, 1 = mBot runs 
int sensorState;
int ultrasonic_distance;

MeDCMotor leftMotor(M1);// left motor connected to M1
MeDCMotor rightMotor(M2);// right motor connected to M1

//instructions for motor
int STOP = 0;
int FORWARD = 1;
int LEFT = 2;
int RIGHT = 3;
int TURN_L = 4;
int TURN_R = 5;
int TURN_180 = 6;

uint8_t Speed = 200; // motor speed 
uint8_t slower_speed = 100; // speed to maintain straight line
uint8_t faster_speed = 170; //speed if too close to the wall

int left_delay = 420; //1 second
int right_delay = 420; //1 second
int delay_180 = 800; //delay twice of one 90degree turn

#define IR 3 //IR input pin at A3
int ir_value;
int ir_count = 0;
int ir_base;
float ir_dist;

#define LDR 2   //LDR sensor pin at A0
#define RGBWait 200 //in milliseconds 
#define LDRWait 10 //in milliseconds 
#define LED 13


//floats to hold colour arrays
float colourArray[] = {0,0,0};
float whiteArray[] = {938, 976,935}; //record down after cali :done
float blackArray[] = {441, 682, 562}; //record down after cali :done
float greyDiff[] = {497, 294, 373};

float calc_ir_dist(int input) {
  float output = 0.00009 * input * input - 0.0969 * input + 29.745;
  return output;
}

void LED_status(int i) {
  if (i == 0) {
    //turn off
    analogWrite(A0, 0);
    analogWrite(A1, 0);
  }
  else if (i == 1) {
    //RED
    analogWrite(A0, 255);
    analogWrite(A1, 255);
  }
  else if (i == 2) {
    //GREEN
    analogWrite(A0, 255);
    analogWrite(A1, 0);
  }
  else {
    //BLUE
    analogWrite(A0, 0);
    analogWrite(A1, 255);
  }
}

void record_baseline_voltage() {
  if (ir_count == 0) {
    LED_status(1); //turn on one led
    ir_base = analogRead(IR);
  } 
  else if (ir_count == 9) {
    ir_count = 0;
  }
  else {
    ir_count++;
  }
}

void colour_checker() {
  if (colourArray[0] >= 220 && colourArray[1] >= 170 && colourArray[2] >= 130) {
    motor_status(TURN_180);
  }
  else if (colourArray[0]>= 220 && colourArray[1] >= 35 && colourArray[2] >= 45) {
    motor_status(TURN_L);
  }
  else if (colourArray[0] >= 140 && colourArray[1] >= 205 && colourArray[2] >= 205) {
    motor_status(8);
  }
  else if (colourArray[0] >= 180 && colourArray[1] >= 250 && colourArray[2] >= 180) {
    motor_status(TURN_R);
  }
  else if (colourArray[0] >= 220 && colourArray[1] >= 150 && colourArray[2]>= 220) {
    motor_status(7);
  }
}

void get_colour() {
  for (int i = 0; i < 3; i++) {
    LED_status(i+1);
    delay(RGBWait);

    colourArray[i] = getAvgReading(5);

    colourArray[i] = (colourArray[i] - blackArray[i]) / (greyDiff[i]) * 255;
    LED_status(0);
    delay(RGBWait);
  }
}

int getAvgReading(int times) {
  int reading;
  int total = 0;
  //take the reading as many times as requested and add them up
  for (int i = 0; i < times; i++)
  {
    reading = analogRead(LDR);
    total = reading + total;
    delay(LDRWait);
  }
  //calculate the average and return it
  return total / times;
}



void motor_stop() {
  leftMotor.stop();  
  rightMotor.stop(); 
}

void motor_forward(){
  leftMotor.run(-Speed);
  rightMotor.run(Speed);
}
void motor_status(int i) {
  //stop
  if (i == 0) {
    leftMotor.stop();  
    rightMotor.stop(); 
  }
  //forward
  else if (i == 1) {
    leftMotor.run(-Speed);
    rightMotor.run(Speed);
  }
  //adjust left
  else if (i == 2) {
    leftMotor.run(-slower_speed);
    rightMotor.run(Speed); 
  }
  //adjust right
  else if (i == 3) {
    leftMotor.run(-Speed);
    rightMotor.run(slower_speed); 
  }
  //turn left
  else if(i == 4) {
    leftMotor.run(Speed);
    rightMotor.run(Speed);
    delay(left_delay);
    motor_stop();
    delay(200);
  }
  //turn right
  else if(i == 5) {
    leftMotor.run(-Speed);
    rightMotor.run(-Speed);
    delay(right_delay);
    motor_status(STOP);
  }
  //turn 180
  else if (i == 6) {
    leftMotor.run(Speed);
    rightMotor.run(Speed);
    delay(delay_180);
    motor_stop();
  }
  //two left turns
  else if (i == 7) {
    leftMotor.run(Speed);
    rightMotor.run(Speed);
    delay(left_delay);
    motor_stop();
    delay(500);
    leftMotor.run(-Speed);
    rightMotor.run(Speed);
    delay(780);
    motor_stop();
    delay(500);
    leftMotor.run(Speed);
    rightMotor.run(Speed);
    delay(right_delay);
    motor_stop();
  }

  //two right turns
  else {
    leftMotor.run(-Speed);
    rightMotor.run(-Speed);
    delay(right_delay);
    motor_stop();
    delay(500);
    leftMotor.run(-Speed);
    rightMotor.run(Speed);
    delay(780);
    motor_stop();
    delay(500);
    leftMotor.run(-Speed);
    rightMotor.run(-Speed);
    delay(right_delay);
    motor_stop();
  }
}

void setBalance() {
  //set white balance
  Serial.println("Put White Sample For Calibration...");
  delay(5000);           //delay for five seconds for getting sample ready
  digitalWrite(LED,LOW); //Check Indicator OFF during Calibration
  //scan the white sample.
  //go through one colour at a time, set the maximum reading for each colour -- red, green and blue to the white array
    for(int i = 0;i<=2;i++){
      LED_status(i+1); //turn on each led
      delay(RGBWait);
      whiteArray[i] = getAvgReading(5); //scan 5 times and return the average, 
      LED_status(0); //turn off led
      delay(RGBWait);
   }
 //done scanning white, time for the black sample.
 //set black balance
   Serial.println("Put Black Sample For Calibration ...");
   delay(5000);     //delay for five seconds for getting sample ready 
 //go through one colour at a time, set the minimum reading for red, green and blue to the black array
   for(int i = 0;i<=2;i++){
      LED_status(i+1); //turn on each led
      delay(RGBWait);
      blackArray[i] = getAvgReading(5);
      LED_status(0); //turn off led
      delay(RGBWait);
 //the differnce between the maximum and the minimum gives the range
      greyDiff[i] = whiteArray[i] - blackArray[i];
   }
 //delay another 5 seconds for getting ready colour objects
   Serial.println("Colour Sensor Is Ready.");
   print_calibration(); // print to the serial monitor the set of white, black and grey for each led
}

void print_calibration() //print the maximum, least possible value and range of value for each led to the serial monitor
{
  for (int i = 0; i < 3; i += 1)
  {
     Serial.print("whiteArray: ");
     Serial.print(whiteArray[i]);
     Serial.print(" blackArray: ");
     Serial.print(blackArray[i]);
     Serial.print(" greyDiff: ");
     Serial.println(greyDiff[i]);
    
  }
}

void setup() {
  Serial.begin(9600);
  LED_status(0);
  //setBalance(); //calibrate ldr before starting
}

void loop() {
  sensorState = lineFinder.readSensors(); // read the line sensor's state 
  record_baseline_voltage();

  if (sensorState == S1_IN_S2_IN) { // situation 1 
    motor_status(STOP); //stop bot
    get_colour(); //read colour
    colour_checker(); //execute action based on colour
  } 
  else {
    LED_status(0); //turn off LED
    ir_value = analogRead(IR);
    ultrasonic_distance = ultraSensor.distanceCm();
    ir_dist = calc_ir_dist(ir_value - ir_base);

    motor_status(FORWARD);

    if (ultrasonic_distance > 25) {
      if (ir_dist < 9.5) { //ir detects that left side of the wall is too close
        motor_status(RIGHT); //adjust right
      }
      else if (ir_dist > 11.5 && ir_dist < 12.5) { //ir detects that right side of the wall is too close
        motor_status(LEFT); //adjust left
      }
      else {
        motor_status(FORWARD); //if both side detects no wall, move forward
      }
    //if too close to right side, move left
    }else if (ultrasonic_distance < 8.5) {
      motor_status(LEFT);
    }
    else {
      motor_status(RIGHT);
    }
  }
}
