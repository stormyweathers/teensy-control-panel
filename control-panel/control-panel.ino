
#include "control-panel.h"

controlPanel panel;

constexpr uint8_t slider_1 = A0, slider_2 = A4;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  panel.init();

  delay(250); // wait for the OLED to power up
  // For some reason, only 8,10,12 work as expected.  All other vals default to 12 bits
  analogReadResolution(8);
  //pinMode(A0,INPUT_PULLDOWN);
}

void loop() {
  // put your main code here, to run repeatedly:
    panel.readState();                         //call as often as possible, around every ms or so
    panel.reportDiff();
    delay(1);
    //panel.enc.tick();
    //if (panel.enc.valueChanged()){
    //    Serial.print("Encoder new value: ");
    //    Serial.println(panel.enc.getValue());
    //}
}
