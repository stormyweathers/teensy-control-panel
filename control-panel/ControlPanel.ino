
#include "ControlPanel.h"
#include "TeensyTimerTool.h"

controlPanel panel;
TeensyTimerTool::PeriodicTimer read_timer(TeensyTimerTool::TCK);

void timer_callback(){
  panel.readState();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
  panel.init();
  // For some reason, only 8,10,12 work as expected.  All other vals default to 12 bits
  panel.resolution=8;
  delay(250); // wait for the OLED to power up
  
  
  Serial.println("init done");

  read_timer.begin(timer_callback,1000);
}

void loop() {
  // put your main code here, to run repeatedly:
    //panel.readState();                         //call as often as possible, around every ms or so
    //panel.reportDiff();
    if (panel.anyInputsChanged){
      panel.reportStateOled(true);
    }
    delay(10);
    //panel.enc.tick();
    //if (panel.enc.valueChanged()){
    //    Serial.print("Encoder new value: ");
    //    Serial.println(panel.enc.getValue());
    //}
}
