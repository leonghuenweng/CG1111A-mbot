#include "MeMCore.h"
MeLineFollower lineFinder(PORT_1); // assigning lineFinder to RJ25 port 2
MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
//int status = 1; // global status; 0 = do nothing, 1 = mBot runs
#define LDR 2   //LDR sensor pin at A0
#define RGBWait 200 //in milliseconds 
#define LDRWait 10 //in milliseconds 
//floats to hold colour arrays
float colourArray[] = {0,0,0};
float whiteArray[] = {250,250,250};
float blackArray[] = {0,0,0};
float greyDiff[] = {0,0,0};

void setup() {
  delay(2);
}

int getAvgReading(int times){      
//find the average reading for the requested number of times of scanning LDR
  int reading;
  int total =0;
//take the reading as many times as requested and add them up
  for(int i = 0;i < times;i++){
     reading = analogRead(LDR);
     total = reading + total;
     delay(LDRWait);
  }
//calculate the average and return it
  return total/times;
}

void loop(){
  leftMotor.run(-101); // Left wheel goes forward (anti-clockwise)
 rightMotor.run(101);
 delay(1000);
 leftMotor.stop(); // Left wheel goes forward (anti-clockwise)
 rightMotor.stop();
 delay(2000);
   analogWrite(A0,255);
  analogWrite(A1,255); //red
  delay(200);
  colourArray[0] = getAvgReading(5);
    analogWrite(A0,255);
  analogWrite(A1,0); //green
  delay(200);
  colourArray[1] = getAvgReading(5);
      analogWrite(A0,0);
  analogWrite(A1,255); //blue
  delay(200);
  colourArray[2] = getAvgReading(5);
  /*for (int c = 0; c <= 2; c++) {
  colourArray[c] = (colourArray[c] - blackArray[c])/(greyDiff[c])*255;
 //show the value for the current colour LED, which corresponds to either the R, G or B of the RGB code
  }*/
  analogWrite(A0,0);
  analogWrite(A1,0); //turn off
  delay(200);

  if (colourArray[0] > 239 && colourArray[0] < 269 ) {
    if (colourArray[1] < 239 && colourArray[1] < 269) {
      if (colourArray[2] <239 && colourArray[2] > 269) {
        // Turning left (on the spot):
          leftMotor.run(100); // Positive: wheel turns clockwise
          rightMotor.run(100); // Positive: wheel turns clockwise
          delay(1500); // Keep turning left for this time duration
      }
    }
  }
}
























//#include "MeMCore.h"
//MeLineFollower lineFinder(PORT_1); // assigning lineFinder to RJ25 port 2
//MeDCMotor leftMotor(M1); // assigning leftMotor to port M1
//MeDCMotor rightMotor(M2); // assigning RightMotor to port M2
//int status = 1; // global status; 0 = do nothing, 1 = mBot runs
//
//#define LDR 2   //LDR sensor pin at A0
//#define RGBWait 200 //in milliseconds 
//#define LDRWait 10 //in milliseconds 
//
////floats to hold colour arrays
//float colourArray[] = {0,0,0};
//float whiteArray[] = {250,250,250};
//float blackArray[] = {0,0,0};
//float greyDiff[] = {0,0,0};
//
//void setup() {
// pinMode(A7, INPUT); // Setup A7 as input for the push button
// Serial.begin(9600); // Setup serial monitor for debugging purpose
//}
//void loop() {
//leftMotor.run(-101); // Left wheel goes forward (anti-clockwise)
// rightMotor.run(101); // Right wheel goes forward (clockwise)
// /*if (analogRead(A7) < 100) { // If push button is pushed, the value will be very low
// status = 1 - status; // Toggle status
// delay(500); // Delay 500ms so that a button push won't be counted multiple times.
// }*/
// if (status == 1) { // run mBot only if status is 1
// int sensorState = lineFinder.readSensors(); // read the line sensor's state
// if (sensorState == S1_IN_S2_IN) { // situation 1
// leftMotor.stop(); 
// rightMotor.stop();
//  delay(200);
// }
// analogWrite(A0,255);
//  analogWrite(A1,255); //red
//  delay(200);
//  colourArray[0] = getAvgReading(5);
//    analogWrite(A0,255);
//  analogWrite(A1,0); //green
//  delay(200);
//  colourArray[1] = getAvgReading(5);
//      analogWrite(A0,0);
//  analogWrite(A1,255); //blue
//  delay(200);
//  colourArray[2] = getAvgReading(5);
//  for (int c = 0; c <= 2; c++) {
//  colourArray[c] = (colourArray[c] - blackArray[c])/(greyDiff[c])*255;
// //show the value for the current colour LED, which corresponds to either the R, G or B of the RGB code
//  }
//  analogWrite(A0,0);
//  analogWrite(A1,0); //turn off
//  delay(200);
//
//  if (colourArray[0] <210 && colourArray[0] > 175) {
//    if (colourArray[1] <179 && colourArray[1] > 139) {
//      if (colourArray[2] <197 && colourArray[2] > 157) {
//        // Turning left (on the spot):
//          leftMotor.run(100); // Positive: wheel turns clockwise
//          rightMotor.run(100); // Positive: wheel turns clockwise
//          delay(1500); // Keep turning left for this time duration
//      }
//    }
//  }
//}
//}
//
//int getAvgReading(int times){      
////find the average reading for the requested number of times of scanning LDR
//  int reading;
//  int total =0;
////take the reading as many times as requested and add them up
//  for(int i = 0;i < times;i++){
//     reading = analogRead(LDR);
//     total = reading + total;
//     delay(LDRWait);
//  }
////calculate the average and return it
//  return total/times;
//}
