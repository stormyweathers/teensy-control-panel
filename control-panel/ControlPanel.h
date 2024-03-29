#ifndef CONTROL_PANEL_H
#define CONTROL_PANEL_H
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "EncoderTool.h"
#include <digitalWriteFast.h>
#include <Bounce2.h>

using namespace EncoderTool;

// Pin definitions
namespace controlPanelBoard{
  //encoder pins
  constexpr uint8_t enc_pinA = 0, enc_pinB = 1, enc_pinBtn = 2;

  // button/switch inputs
  constexpr uint8_t joystick_button=11, button=12, toggle=13;
  constexpr uint8_t digital_in[] = {joystick_button,button,toggle};

  //Analog slider/pot inputs
  constexpr uint8_t joystick_x = A0, joystick_y = A1, joystick_z = A4;
  constexpr uint8_t slider_r = A5, slider_c = A6, slider_l = A7;
  // unused ports on the board: A8,A9
  constexpr uint8_t analog_in[] = {joystick_x,joystick_y,joystick_z,slider_r,slider_c,slider_l}; 

  //Pins for oled screen
  constexpr uint16_t screen_width=128, screen_height=128;
  constexpr int8_t oled_reset=-1;

  //Pins for digital outputs
  constexpr uint8_t digital_out_0[] = {3,4,5,6};
  constexpr uint8_t digital_out_1[] = {7,8,9,10}; 
  //constexpr uint8_t digital_out_all[] = {3,4,5,6,7,8,9,10};
  //using 22,23 as digital outs instead of analog outs
  constexpr uint8_t digital_out_all[] = {3,4,5,6,7,8,9,10,22,23};
  constexpr uint8_t num_digital_outs = 8;
}

class controlPanel {
  public:

    int resolution;

    //Constructor
    controlPanel();
    

    // Initializer
    void init();
    void init(uint8_t rotation);
    

    //Read all inputs
    void readState();

    // serial print the difference between current and previous state
    void reportDiff();

    // write the state of all inputs to OLED
    void reportStateOled(bool update_display);

    //
    void poll();

    //debounce objects for digital inputs
    Bounce2::Button joystick_button = Bounce2::Button();
    Bounce2::Button button = Bounce2::Button();
    Bounce toggle = Bounce();

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

    // neutral values, may change over long time
    int joystick_x0;
    int joystick_y0;
    int joystick_z0;

    // Converted to floats
    float joystick_x;
    float joystick_y;
    float joystick_z;

    //write to the digital outputs
    bool dWrite(uint8_t,bool);

    //Retreive pin number from digital output ports
    uint8_t getPinNumber(uint8_t);

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

//overload initializer to give optional rotation parameter
void controlPanel::init(uint8_t rotation){
  controlPanel::init();
  display.setRotation(rotation); 
}

void controlPanel::init(){

  enc.begin(controlPanelBoard::enc_pinA, controlPanelBoard::enc_pinB, controlPanelBoard::enc_pinBtn);  
  display.begin(0x3D, true);
  delay(250); // wait for the OLED to power up
  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SH110X_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println("Teensy Control Panel Booting up!");
  display.display();

  // Initialize debounce objects for digital inputs
  button.attach(controlPanelBoard::button,INPUT_PULLUP);
  button.interval(25);
  button.setPressedState(LOW);

  joystick_button.attach(controlPanelBoard::joystick_button,INPUT_PULLUP);
  joystick_button.interval(25);
  joystick_button.setPressedState(LOW);

  toggle.attach(controlPanelBoard::toggle,INPUT_PULLUP);
  toggle.interval(25);

  analogReadResolution(this->resolution);
  //Set default values at halfway point
  this->joystick_x0 = pow(2,this->resolution-1)-1;
  this->joystick_y0 = pow(2,this->resolution-1)-1;
  this->joystick_z0 = pow(2,this->resolution-1)-1;
}

void controlPanel::poll(){
  //Poll all digital input pins
  
  //Encoder
  enc.tick();
  encoderChanged = enc.valueChanged();
  encoderButtonChanged = enc.buttonChanged();

  joystick_button.update();
  button.update();
  toggle.update();
  digitalInChanged = joystick_button.changed() | button.changed() | toggle.changed();

}

void controlPanel::readState(){
  //Read states of all inputs, cache old state, and compare to detect changes
  poll();

  analogInChanged = 0;
  //analog_inputs
  analogInChanged = 0;
  for (uint8_t idx=0; idx<sizeof(controlPanelBoard::analog_in)/sizeof(controlPanelBoard::analog_in[0]);idx++ ){
    analog_in_state_cache[idx] = analog_in_state[idx];
    analog_in_state[idx] = analogRead(controlPanelBoard::analog_in[idx]);
    //single bit changes are common due to fluctuations, set a diff threshold
    analogInChanged |= (  0 < abs( static_cast <int16_t> (analog_in_state[idx]) - static_cast<int16_t>(analog_in_state_cache[idx])   )    );
  }

  anyInputsChanged = encoderChanged | encoderButtonChanged | analogInChanged | digitalInChanged;

  this->joystick_x = (this->analog_in_state[0]-this->joystick_x0)/pow(2,this->resolution-1);
  this->joystick_y = (this->analog_in_state[1]-this->joystick_y0)/pow(2,this->resolution-1);
  this->joystick_z = (this->analog_in_state[2]-this->joystick_z0)/pow(2,this->resolution-1);
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

void controlPanel::reportStateOled(bool update_display){
  if(update_display){
  display.clearDisplay();
  display.setCursor(0,0);
  }

  display.print("Encoder: (  ");
  display.print(enc.getValue());
  display.print(" , ");
  display.println(enc.getButton() == LOW ? "X )" : "O )");

  display.print("Button: ");
  display.println(button.isPressed() == HIGH ? "X" : "O");

  display.print("Toggle: ");
  display.println(toggle.read() == LOW ? "X" : "O");

  display.print("Joystick: ");
  display.print(analog_in_state[0]);
  display.print(" ");
  display.print(analog_in_state[1]);
  display.print(" ");
  display.println(analog_in_state[2]);

  display.print("Sliders: ");
  display.print(analog_in_state[5]);
  display.print(" ");
  display.print(analog_in_state[4]);
  display.print(" ");
  display.println(analog_in_state[3]);

  if (update_display){display.display();}
}

bool controlPanel::dWrite(uint8_t port_num, bool state){
  if (port_num < controlPanelBoard::num_digital_outs){
    digitalWriteFast(controlPanelBoard::digital_out_all[port_num],state);
    return 1;
  }
  else{
    Serial.print("ctrl panel invalid write to port: ");
    Serial.println(port_num);
    return 0;
  }
}

uint8_t controlPanel::getPinNumber(uint8_t port_num){
  return controlPanelBoard::digital_out_all[port_num];
}

#endif //CONTROL_PANEL_H