#include "MeMCore.h"

#define IR 3 //IR input pin at A3
#define LDR 2   //LDR sensor pin at A0
#define RGBWait 200 //in milliseconds 
#define LDRWait 10 //in milliseconds 
#define LED 13 //Indicator on mBot arduino

MeLineFollower lineFinder(PORT_1); // linefollower sensor connected to port 1
MeUltrasonicSensor ultraSensor(PORT_2); // ultrasonic sensor connected to port 2
MeDCMotor leftMotor(M1);// left motor connected to M1
MeDCMotor rightMotor(M2);// right motor connected to M1

int status = 0; // global status; 0 = do nothing, 1 = mBot runs 
int sensorState; //input for the line sensor
int ultrasonic_distance; //distance measured from ultrasonic sensor

//instructions for motor
int LEFT = 0; //adjust left when going straight
int RIGHT = 1; //adjust right when going straight
int TURN_L = 2; //90 degree left turn
int TURN_R = 3; //90 degree right turn
int TURN_180_L = 4; //180 anti clockwise turn
int TURN_180_R = 5; //180 clockwise turn

uint8_t Speed = 200; // motor speed 
uint8_t slower_speed = 160; // speed of motor that is further away from wall
uint8_t faster_speed = 210; //speed of motor that is closer to the wall

int left_delay = 398; //time taken to make left turn
int right_delay = 380; //time taken to make right turn
int delay_180 = 719; //180 turn delay

int ir_value; //input reading from IR detector
int ir_count = 0;
int ir_base; //stores the base IR reading when detector is off
float ir_dist; //the distance reading from IR

//floats to hold colour arrays
float colourArray[] = {0,0,0};
float whiteArray[] = {970,938,860}; //record down after cali :done
float blackArray[] = {843,762,619}; //record down after cali :done
float greyDiff[] = {127,176,241};

MeBuzzer buzzer;
char i;
int note, duration;

void record_baseline_voltage() {
  if (ir_count == 0) {
    LED_status(1); //turn on one led to turn off IR detector
    ir_base = analogRead(A3); //measure base line voltage
  } 
  else if (ir_count == 9) { 
    ir_count = 0; //reset count on 10th iteration
  }
  else {
    ir_count++;
  }
}

void LED_status(int i) {
  if (i == 0) {
    //turn off
    analogWrite(A0, 0);
    analogWrite(A1, 0);
  }
  else if (i == 1) {
    //turn on RED
    analogWrite(A0, 255);
    analogWrite(A1, 255);
  }
  else if (i == 2) {
    //turn on GREEN
    analogWrite(A0, 255);
    analogWrite(A1, 0);
  }
  else {
    //turn on BLUE
    analogWrite(A0, 0);
    analogWrite(A1, 255);
  }
}

void colour_checker() {
  if (colourArray[0] >= 235 && colourArray[1] >= 235 && colourArray[2] >= 235) {
    motor_stop(); //white detected
    play_tune();
  }
  else if (colourArray[0] > colourArray[1] && colourArray[0] > colourArray[2]) { //red orange purple will have the highest 'r' value
    if (colourArray[0] < 210) { //only purple Red will be below 210
      motor_status(6); // purple detected, two successive left turns
    } else if (colourArray[1] > 107) { //orange
      if (ultrasonic_distance > 12) { //if closer to left wall, turn clockwise
        motor_status(TURN_180_R);
      } else {
        motor_status(TURN_180_L); //turn 180 anti clockwise
      }
    } else {
      motor_status(TURN_L); //red, turn left
    }
  }
  else if (colourArray[1] > colourArray[0] && colourArray[1] > colourArray[2]) { // only green has the highest green value
    motor_status(TURN_R); // green detected, turn right
  }
  else if (colourArray[2] > colourArray[0] && colourArray[2] > colourArray[1]) { //only blue has the highest blue value
    motor_status(7); //two successive right turns
  } else {
     get_colour(); //read LDR again if colour unsure
  }
}


void get_colour() {
  for (int i = 0; i < 3; i++) {
    LED_status(i+1); //turn on led one by one
    delay(RGBWait);

    colourArray[i] = getAvgReading(5); //read LDR

    colourArray[i] = (colourArray[i] - blackArray[i]) / (greyDiff[i]) * 255;
    LED_status(0); //turn OFF led
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

//stop
void motor_stop() {
  leftMotor.stop();  
  rightMotor.stop(); 
}

//move forward
void motor_forward() {
  leftMotor.run(-Speed);
  rightMotor.run(Speed);
}

void motor_status(int i) {
  //adjust left
  if (i == 0) {
    leftMotor.run(-slower_speed);
    rightMotor.run(faster_speed); 
  }
  //adjust right
  else if (i == 1) {
    leftMotor.run(-210);
    rightMotor.run(160); 
  }
  //turn left
  else if(i == 2) {
    leftMotor.run(Speed);
    rightMotor.run(Speed);
    delay(left_delay);
    motor_stop();
  }
  //turn right
  else if(i == 3) {
    leftMotor.run(-Speed);
    rightMotor.run(-Speed);
    delay(right_delay);
    motor_stop();
  }
  //turn 180 left
  else if (i == 4) {
    leftMotor.run(Speed);
    rightMotor.run(Speed);
    delay(delay_180);
    motor_stop();
  }
  //turn 180 right
  else if (i == 5) {
    leftMotor.run(-Speed);
    rightMotor.run(-Speed);
    delay(delay_180);
    motor_stop();
  }
  //two left turns (purple)
  else if (i == 6) {
    //turn left
    leftMotor.run(Speed);
    rightMotor.run(Speed);
    delay(393);
    motor_stop();
    delay(500);
    //move forward
    leftMotor.run(-Speed);
    rightMotor.run(Speed);
    delay(841);
    motor_stop();
    delay(500);
    //turn left
    leftMotor.run(Speed);
    rightMotor.run(Speed);
    delay(373);
    motor_stop();
  }
  //two right turns
  else {
    //turn right
    leftMotor.run(-Speed);
    rightMotor.run(-Speed);
    delay(397);
    motor_stop();
    delay(500);
    //move forward
    leftMotor.run(-Speed);
    rightMotor.run(Speed);
    delay(802);
    motor_stop();
    delay(500);
    //turn right
    leftMotor.run(-Speed);
    rightMotor.run(-Speed);
    delay(378);
    motor_stop();
  }
}

// plays the tune required when detecting white after stopping in front of black paper
void play_tune()
{                
  for (i = 0; i < 10; i ++)
  {
    note = random(100, 1500); // Freq range of numbers
    duration = random(50, 300); // Duration for each notes
    buzzer.tone(note, duration);
  }
  motor_stop();
  status = 0;
  delay(200);
}

void setBalance() {
  //set white balance
  Serial.println("Put White Sample For Calibration...");
  delay(5000);           //delay for five seconds for getting sample ready
  digitalWrite(LED,LOW); //Check Indicator OFF during Calibration
  
  //scan the white sample 
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
   
   //scan the black sample
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

//print the maximum, least possible value and range of value for each led to the serial monitor
void print_calibration() 
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
  motor_stop();
  //set_balance(); //only used when we want to calibrate to get white, black and grey array values
}

void loop() {
  if (analogRead(A7) < 100) { //when button is pushed, input is very low
    status = 1; //status 1 allows bot to run
    delay(500); //prevent from reading A7 again
  }
  
  if (status == 1) {
    sensorState = lineFinder.readSensors(); // read the line sensor's state 
    record_baseline_voltage(); //baseline value of IR

    if (sensorState == S1_IN_S2_IN) { // situation 1 
      motor_stop(); //stop bot
      get_colour(); //read colour
      colour_checker(); //execute action based on colour
    } 
    else {
      LED_status(0); //turn off LED, IR detector is turned on
      ir_value = analogRead(A3);
      ultrasonic_distance = ultraSensor.distanceCm();
      ir_dist = ir_base - ir_value; //calculate ir distance
    
      motor_forward();

      if (ultrasonic_distance > 25) { //when no wall on ultrasonic side, activate IR
        if (ir_dist < -80) { //ir detects that left side of the wall is too close
          motor_status(RIGHT); //adjust right
        }
        else if (ir_dist > -80  && ir_dist < -65) { //ir detects that right side of the wall is too close
          motor_status(LEFT); //adjust left
        }
        else {
          motor_forward(); //if both side detects no wall, move forward
        }
      } 
      //if too close to right side, move left
      else if (ultrasonic_distance < 10) {
          motor_status(LEFT);
      }
      else {
        motor_status(RIGHT); //adjust right
      }
    }
  }
}
