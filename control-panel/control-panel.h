#ifndef CONTROL_PANEL_H
#define CONTROL_PANEL_H
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "EncoderTool.h"
#include <digitalWriteFast.h>
using namespace EncoderTool;

// Pin definitions


namespace controlPanelBoard{
  //encoder pins
  constexpr uint8_t enc_pinA = 0, enc_pinB = 1, enc_pinBtn = 2;

  // button/switch inputs
  constexpr uint8_t digital_in[] = {11,12,13};

  //Analog slider/pot inputs
  constexpr uint8_t analog_in[] = {A0,A1,A4,A5,A6,A7,A8, A9}; 

  //Pins for oled screen
  constexpr uint16_t screen_width=128, screen_height=128;
  constexpr int8_t oled_reset=-1;

  //Pins for digital outputs
  constexpr uint8_t digital_out_0[] = {3,4,5,6};
  constexpr uint8_t digital_out_1[] = {7,8,9,10}; 
  constexpr uint8_t digital_out_all[] = {3,4,5,6,7,8,9,10};

}

class controlPanel {
  public:
    //Constructor
    controlPanel();

    // Initializer
    void init();

    //Read all inputs
    void readState();

    // serial print the difference between current and previous state
    void reportDiff();

    //flags to detect changed inputs
    bool anyInputsChanged;
    bool encoderChanged;
    bool encoderButtonChanged;
    bool digitalInChanged;
    bool analogInChanged;

    PolledEncoder enc;
    Adafruit_SH1107 display = Adafruit_SH1107(controlPanelBoard::screen_width, controlPanelBoard::screen_height, &Wire1, controlPanelBoard::oled_reset, 1000000, 100000);

    //Report the input states
    bool digital_in_state[3];
    uint16_t analog_in_state[8];
    bool digital_out_state[8];

  private:
    //Previous state for noticing changes
    bool digital_in_state_cache[3];
    uint16_t analog_in_state_cache[8];
    bool digital_out_state_cache[8];
};


//Constructor
controlPanel::controlPanel(){

  // Initialize digital outputs and set to zero ASAP
  // Dont want to have anything turned on by mistake at startup
  for (uint8_t idx=0; idx< sizeof(controlPanelBoard::digital_out_all)/sizeof(controlPanelBoard::digital_out_all[0]); idx++){
    pinModeFast(controlPanelBoard::digital_out_all[idx], OUTPUT);
    digitalWriteFast(controlPanelBoard::digital_out_all[idx], 0);
  }

}

void controlPanel::init(){

  for (uint8_t idx=0; idx<sizeof(controlPanelBoard::digital_in)/sizeof(controlPanelBoard::digital_in[0]);idx++ ){
    pinMode(controlPanelBoard::digital_in[idx],INPUT_PULLUP);
  }


  enc.begin(controlPanelBoard::enc_pinA, controlPanelBoard::enc_pinB, controlPanelBoard::enc_pinBtn);

  delay(250); // wait for the OLED to power up
  display.begin(0x3D, true);
  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SH110X_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println("Teensy Control Panel Booting up!");
  display.display();


}

void controlPanel::readState(){
  //Read states of all inputs, cache old state, and compare to detect changes
  analogInChanged = 0;

  //Encoder
  enc.tick();
  encoderChanged = enc.valueChanged();
  encoderButtonChanged = enc.buttonChanged();

  //Digital inputs
  digitalInChanged = 0;
  for (uint8_t idx=0; idx<sizeof(controlPanelBoard::digital_in)/sizeof(controlPanelBoard::digital_in[0]);idx++ ){
    digital_in_state_cache[idx] = digital_in_state[idx];
    digital_in_state[idx] = digitalRead(controlPanelBoard::digital_in[idx]);
    digitalInChanged |= (digital_in_state[idx] != digital_in_state_cache[idx]);
  }

  //analog_inputs
  analogInChanged = 0;
  for (uint8_t idx=0; idx<sizeof(controlPanelBoard::analog_in)/sizeof(controlPanelBoard::analog_in[0]);idx++ ){
    analog_in_state_cache[idx] = analog_in_state[idx];
    analog_in_state[idx] = analogRead(controlPanelBoard::analog_in[idx]);
    //single bit changes are common due to fluctuations, set a diff threshold
    analogInChanged |= (  0 < abs( static_cast <int16_t> (analog_in_state[idx]) - static_cast<int16_t>(analog_in_state_cache[idx])   )    );
  }

  anyInputsChanged = encoderChanged | encoderButtonChanged | analogInChanged | digitalInChanged;
}


void controlPanel::reportDiff(){
    if (encoderChanged)             // do we have a new encoder value?
    {
        Serial.print("Encoder new value: ");
        Serial.println(enc.getValue());
    }
    if (encoderButtonChanged)            // do we have a new button state?
    {
        Serial.print("Encoder button state: ");
        Serial.println(enc.getButton() == LOW ? "pressed" : "released");
    }

    if (analogInChanged ){
      Serial.print("Analog input vals: ");
      for (uint8_t idx=0; idx<sizeof(analog_in_state)/sizeof(analog_in_state[0]);idx++ ){
        Serial.print("  ");
        Serial.print(analog_in_state[idx]);
        Serial.print("  ");
      }
      Serial.println("");
    }

    if (digitalInChanged){
      Serial.print("Digital input vals: ");
      for (uint8_t idx=0; idx<sizeof(digital_in_state)/sizeof(digital_in_state[0]);idx++ ){
        Serial.print("  ");
        Serial.print(digital_in_state[idx]);
        Serial.print("  ");
      }
      Serial.println();
    }


}
#endif //CONTROL_PANEL_H