#include "MeMcore.h"

int ir_value;
int ir_count = 0;
int base_ir;
float ir_dist;

void loop() {
  record_baseline_voltage();

  ir_value = analogRead(A3);
  ir_dist = calc_ir_distance(ir_value - base_ir);

  if (ir_dist < 6.5) { //ir detects mbot close to right side of the wall
    turn_left_slowly(); // move left
  }
  else if (ir_dist > 10.5 && ir_dist < 11.5) { //ir detects mbot close to left side of the wall
    turn_right_slowly(); //move right
  }
  else {
    move_forward(); // both side detects no wall, move forward
  }
}

float calc_ir_dist(int input) {
  float output = 0.00009 * input * input - 0.0969 * input + 29.745;
  return output;
}

void record_baseline_voltage() {
  if (ir_count == 0) {
    turn_on_LED(1);
    base_ir = analogRead(A3);
  } 
  else if (ir_count == 9) {
    ir_count = 0;
  }
  else {
    ir_count++;
  }
}
