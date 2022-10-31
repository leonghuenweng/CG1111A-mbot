
void colour_checker() {
  
}
void read_ldr() {
  for (int i = 0; i < 3; i++) {
    turn_on_LED(i);
    delay(RGBWait);

    colourArray[i] = getAvgReading(5);

    colourArray[c] = (colourArray[c] - blackArray[c]) / (greyDiff[c]) * 255;
    turn_off_LED();
    delay(RGBWait);
  }
}

int getAvgReading() {
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
