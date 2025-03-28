//FINAL MASTER THESIS: Master's degree in Aeronautical Engineering
// TITLE:               
// CONTACT:             Manuel António Caetano Azevedo - manuelcaetano40@gmail.com - https://www.linkedin.com/in/im-manuel-azevedo/
// UNIVERSITY:          UBI, Faculdade de Engenharia

//--------------------------------------------- Code structure and information ----------------------------------------------------
// This code was made to control and collect data from an Engine Test Bench (ETB). The ETB was developed as a master's dissertation
// at University of Beira Interior. The ETB will have, in this version, 2 different modes of operation, "Static" and "Manual". The 
// static mode is automatic with just the selection of the initial load and minimum rpm needed. On the "manual" mode the test is done
// manually with the user having to select the load.
// The code is structured in levels on loop() and steps on the different modes functions. Every time a task is completed the step or 
// level value is incremented until it is needed to return to a previous level/step.


#include "HX711.h"                                                        // Library used to get data from the HX711 amplifier
#include <Wire.h>
#include <LiquidCrystal_I2C.h>                                            // Library used for I2C module and LCD
#include <string.h>                                                       // Library used to better manipulate strings
#include <Adafruit_INA219.h>                                              // Library for use of the INA219 current sensor module
#include <SPI.h>
#define seconds() (millis()/1000.0)                                       

// Load Cell for measurement of engine torque with HX711 module
const int LOADCELL_DOUT_PIN = 10;                                          // Pin connected to DOUT of HX711
const int LOADCELL_SCK_PIN = 11;                                           // Pin connected to SCK of HX711 
HX711 scale_engine;                                                              // Start link with HX711
float Calibration = 431.31;                                                // Calibration for load cell
float Distance = 0.40;                                                    // Distance from the load cell to the axle
float Mass;                                                               // Mass measured in grams
float Weight;                                                             // Weight measured in Newtons
float Torque;                                                             // Engine torque in N.m

// Load Cell for fuel consumption measurement with HX711 module
const int fuel_cell_DT_pin = 13;                                          // Pin connected to DOUT
const int fuel_cell_SCK_pin = 12;                                         // Pin connected to SCK
HX711 scale_fuel;                                                         // Start link with HX711
float fuel_calibration = 431.31;                                          // Calibration for the load cell
float fuel_mass;                                                          // fuel mass measured in grams


// RPM measurements
const int rpmPin = 2;                                                     // Pin 2 connected to hall sensor signal
int rpm;                                                                  // RPM value to be displayed
float PrevTime = 0;                                                       // Time of previous magnet detection in microseconds
float Duration = 0;                                                       // Time elapsed between magnet detection in microseconds
double time = 0;                                                          // Used to count 60 seconds from the start of the test
float time_abs = 0;                                                       // Variable used to record the absolute time in the "Collect_Data()" function
float real_rpm = 0;

//Power Measurements
float Total_Power = 0;                                                    // Total power delivered to the axle
float Bat_Power = 0;                                                      // Electric power delivered by the battery to the Esc

// Buttons - these variables are used to avoid that noise has any type of influence on the push buttons
int lastButtonState = LOW;
int buttonState = LOW;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;
unsigned long buttonPressStartTime = 0;                                   // Time when button press started
const unsigned long pressDuration = 5000;                                 // 5 seconds in milliseconds
bool buttonPressed = false;                                               // To keep track of the button state
bool buttonPressedFor5Seconds = false;                                    // To keep track if the button has been pressed for 5 seconds

//Selectors - variables used for selecting the mode of operation, the initial load and the minimum rpm
int menuIndex = 0;
const int numMenuItems = 2;
int initial_current = 0;
int rpm_min = 0;
int new_rpm_min = 0;
int rpm_adjusted = 0;
int level = 1;
int step = 0;
int minInput = 1023;
int maxInput = 0;
int minOutput = 0;
int maxOutput = 4000;
int increment = 100;

//Current Sensor variables
Adafruit_INA219 ina219_coil(0x40);
Adafruit_INA219 ina219_motor(0x41);
float current_load = 0;
float current_load2 = 0;
float average_current = 0;
float sum = 0;

// Data collection
float time_now = 0;                                                     // Time at the present reading
char type[7];                                                           // String to know which type of test it was done
int reps = 0;                                                           // Variable to know if the header is printed or not

//Joystick variables
const int upPin = 9;                                                   // Pin connected to the "up" button
const int downPin = 8;                                                 // Pin connected to the "down" button
const int selectPin = 3;                                               // Pin connected to the "select" button
const int setPin = 7;                                                  // Pin connected to the "set" button
int lastTareButtonState = HIGH; // Assuming active-low configuration
const int rstPin = 6;                                                   // Pin connected to the "Reset" button

//Digital potentiometer initialization
int CS_PIN = 49;                                                        // Chip Select pin for MCP4231
int wiperValue = 127;                                                     // Stores the current wiper position (0-127 for MCP4231)


LiquidCrystal_I2C lcd(0x27, 16, 2);

int mapToIncrements(int value, int inMin, int inMax, int outMin, int outMax, int increment) {
    int range = (outMax - outMin) / increment;
    int mappedValue = map(value, inMin, inMax, 0, range);
    return outMin + (mappedValue * increment);
}

void setup() {
  lcd.init();                                                   // Start I2C link with LCD
  lcd.backlight();                                              // Turn on LCD backlight

  lcd.setCursor(0,0);
  lcd.print("Starting checks");
    
  // Definition of pin modes and activation of internal resistors
  pinMode(upPin, INPUT_PULLUP);                                         // "Up" button on the joystick
  pinMode(downPin, INPUT_PULLUP);                                       // "Down" button on the joystick
  pinMode(selectPin, INPUT_PULLUP);                                     // Button used to select and confirm modes and values
  pinMode(setPin, INPUT_PULLUP);                                        // Button used to tare the scale
  pinMode(rstPin, INPUT_PULLUP);                                        // Emergency stop button
  pinMode(rpmPin, INPUT_PULLUP);                                        // pin used for the Hall sensor signal
  
  attachInterrupt(digitalPinToInterrupt(rpmPin), countRpm, RISING);     // Interrupt used to call function countRPM() everytime there is
                                                                        // signal from the Hall Sensor
  
  scale_engine.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);              // Start link with HX711
  scale_engine.tare();                                                  // Tare scale to zero
  scale_engine.set_scale(Calibration);                                  // Adjust scale to calibration factor

  scale_fuel.begin(fuel_cell_DT_pin, fuel_cell_SCK_pin);                // Start link with HX711
  scale_fuel.tare();                                                    // Tare fuel scale to 0
  scale_fuel.set_scale(fuel_calibration);                               // Adjust scale to calibration factor

  ina219_coil.setCalibration_32V_2A();                                  //Calibrate the ina219 modules for the shunts used (50A, 75mV)
  ina219_motor.setCalibration_32V_2A();                                 // for this custom calibration some changes had to be made to the library
                                                                        // (see Adafruit_INA219.cpp)

  Serial.begin(9600);
  delay(1000);

  lcd.clear();  
  lcd.setCursor(0, 0);
  lcd.print(" The Dyno Bench ");                                // Delay routine to stabilize scale.tare
  lcd.setCursor(0, 1);
  lcd.print("   C-MAST UBI   ");
  delay(1500);
  updateMenu();

  SPI.begin();
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  setWiper(0);                                                  // Initialize wiper position
}

void loop() {
    // In the loop() only the main things are done. These includes selecting the mode of operation, Tare the load cell
    // and emergency stoping the ETB. In "level" 1 the joystick is used to walk trough the different modes and when
    // the middle button is pressed the mode is selected and "level" is incremented to 2. When this happen the select-
    // ed mode function is called.

    
  if (level == 1) {
    // Check the "up" button
    if (digitalRead(upPin) == LOW && millis() - lastDebounceTime > debounceDelay) {
      lastDebounceTime = millis();
      menuIndex--;
      if (menuIndex < 0) menuIndex = numMenuItems - 1;  // Wrap around to last item
      updateMenu();
    }

    // Check the "down" button
    if (digitalRead(downPin) == LOW && millis() - lastDebounceTime > debounceDelay) {
      lastDebounceTime = millis();
      menuIndex++;
      if (menuIndex >= numMenuItems) menuIndex = 0;  // Wrap around to first item
      updateMenu();
    }

    if (digitalRead(selectPin) == LOW && millis() - lastDebounceTime > debounceDelay) {
      lastDebounceTime = millis();
      // Action for the selected menu option
      selectMenuItem();
      level = 2;
    }
  }
  else if (level == 2) {
    if (menuIndex == 0) {
      estatico();
    } else if (menuIndex == 1) {
      manual();
    }
  }

    
  // In each loop, the tare and emergency buttons are checked. If any of them is pressed,
  // according action is taken.
    
  int tareButtonState = digitalRead(setPin);  // Read the button on pin 43
  
  // Check for button press (high to low transition)
  if (lastTareButtonState == HIGH && tareButtonState == LOW) {
    scale_engine.tare();  // Tare the load cell
    scale_fuel.tare();  
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Load Cells Tared");
    delay(1000);  // Short delay to show feedback on the LCD
    updateMenu(); // Refresh the menu after taring
  }
  
  lastTareButtonState = tareButtonState; // Update the last state

  // Check if Emergency Stop button (pin 41) is pressed
  if (digitalRead(rstPin) == LOW) {
    delay(50); // Debounce delay
    if (digitalRead(rstPin) == LOW) { // Confirm press
      // Stop all processes, reset states, and return to main menu
      level = 1; 
      step = 0; 
      Pot.set(0); // Reset potentiometer to 0
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Emergency stop");
      delay(2000); // Show message for 2 seconds
      updateMenu(); // Go back to the main menu
    }
  }
}


void updateMenu() {
  // This function prints to the LCD the Current Menu option. Each time the joystick is used
  // "menuIndex" value changes and the different option appear on the bottom line of the LCD  
    
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Menu: ");
  
  switch (menuIndex) {
    case 0:
      lcd.setCursor(0, 1);
      lcd.print("1 - Estatico");
      break;
    case 1:
      lcd.setCursor(0, 1);
      lcd.print("2 - Manual");
      break;
  }
}

void selectMenuItem() {
  // Actions for when the "middle" button of the joystick is pressed, selecting the mode of
  // operation 
    
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Selecionado!");
  lcd.setCursor(0,1);
  delay(1000);
  lcd.clear();
}

void estatico()
{
  // This function corresponds to the static mode of testing. In this mode, the first var-
  // iables, Current and RPM are selected. To select these values, a joystick is used.
  // After each value is selected the value "step" is incremented and the function conti-
  // nues. 
  if(step == 0){
    current();
    step = 1;
  }
  else if (step == 1)
  {
  int new_initial_current = initial_current;  // Initialize to avoid jumps
  int upState, downState, selectState;

  // Read the joystick directions and button state
  upState = digitalRead(upPin);      
  downState = digitalRead(downPin);  
  selectState = digitalRead(selectPin);

  // Adjust initial_current based on up/down movement
  if (upState == LOW && initial_current < 30) {  // Joystick up pressed
    new_initial_current++;
  } else if (downState == LOW && initial_current > 0) {  // Joystick down pressed
    new_initial_current--;
  }

  // Update the display only if initial_current changed
  if (new_initial_current != initial_current) {
    initial_current = new_initial_current;
    current();  // Call to update the display or handle current
  }

  // Debouncing for select button to confirm selection
  if (selectState != lastButtonState) {
  lastDebounceTime = millis();  // Reset debounce timer
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (selectState != buttonState) {
      buttonState = selectState;
      if (buttonState == LOW) {  // Joystick select button pressed
        step = 2;
        //show "Selecionado!" on LCD
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Selecionado!");
      }
    }
  }
  lastButtonState = selectState;  // Update last button state
  delay(250);  // Small delay for processing
  // Initialize new_rpm_min at the beginning
  }
  else if (step == 2)
  {
    rpm_selecter();
    // Check joystick states
    int upState = digitalRead(upPin);      // Joystick up pin
    int downState = digitalRead(downPin);  // Joystick down pin
    int selectState = digitalRead(selectPin); // Joystick select pin

    // Adjust new_rpm_min based on joystick movement
    if (upState == LOW) {  // Joystick up pressed
      new_rpm_min += 50;  // Increment by 50
    } else if (downState == LOW) {  // Joystick down pressed
      if (new_rpm_min >= 50) {   // Prevent going below 0
        new_rpm_min -= 50;  // Decrement by 50
      }
    }

    // Update the display only if new_rpm_min changed
    if (new_rpm_min != rpm_min) {  // Check if there's a change
      rpm_min = new_rpm_min;
      rpm_selecter();  // Update the display or handle the new rpm_min
    }

    // Debouncing for select button to confirm selection
    if (selectState != lastButtonState) {
      lastDebounceTime = millis();  // Reset debounce timer
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
      if (selectState != buttonState) {
        buttonState = selectState;
        if (buttonState == LOW) {  // Joystick select button pressed
          step = 3;
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Selecionado!");
          delay(1000);  // Delay to show confirmation message
        }
      }
    }
    lastButtonState = selectState;  // Update last button state
    delay(250);  // Small delay for processing
  }
  else if(step == 3)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Teste a comecar!");
    lcd.setCursor(0,1);
    lcd.print("Set throttle!");
    //Serial.println("O teste vai ser começar!");
    //Serial.println("Garante que o acelerador está na posição a testar");
    step = 4;
  }
  else if(step == 4)
  {
    // POWER SUPPLY START-UP
    // The start-up of the Power Supply is done through the potentiometer.
    // With the current value selected, an average of 10 current values is conducted
    // to ensure that the current on the coil is the selected.
      
    sum = 0;   // Reset sum to zero before starting

    for (int i = 1; i <= 10; i++) {
      
      current_load = ina219_coil.getShuntVoltage_mV()*50/75;               // Calculate the current

      sum += current_load;                                            // Accumulate current values
      delay(50);                                                      // Short delay between readings to smooth out noise
    }

    average_current = sum / 10;                                       // Calculate the average current

    // Adjust potentiometer based on the average current
    if (average_current - initial_current < -0.2) {                 // If average current is less than target (below threshold)
      decreasePot(1);                                                // Increment potentiometer
    } else if (average_current - initial_current > 0.2) {           // If average current is greater than target (above threshold)
      increasePot(1);                                              // Decrement potentiometer
    } else {
    // when the average is equal to the value selected, "step" is increment and the sequence continues
      step = 5;
      time = millis();
      time_abs = millis();
    }
 
  }
  else if(step == 5)
  {
    // During one minute the function "Collect_Data()" is called and data is collected
      
    if (millis() - time < 60000)
    {
      strcpy(type, "static");
      
      Collect_Data();
    }
    else
    {
      step = 6;
    }
  }
  else if(step == 6)
  {
    // the load is incremented by one step and data continues to be collected
    // in order to see the response of the engine to the change of load  
    
    int load_increment = 1;
    increasePot(1); 
    
    step = 7;
    Collect_Data();
    time = millis();
  }
  else if(step == 7)
  {
    // for 2 seconds data is collected which gives time for the engine to stabilize
    // on the new load
      
    if(millis() - time < 2000){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("A testar limite.");
      Collect_Data();
    }

    RPM();                    // the rpm of the engine under the new load is calculated
    int rpm_test = rpm*4;

    Collect_Data();

    if (rpm_min < rpm_test)   // if the rpm of the engine is greater than the minimum selected
    {                         // the test sequence may continue, returning to step 5
      step = 5;
      time = millis();
    }
    else                      // if the rpm of the engine is lower than the value selected, the
    {                         // the test has to be stopped to avoid stalling/damaging the engine.
      level = 1;              // "level" and "step" must return to 1 and 0 respectively and the di 
      step = 0;               // gital potentiometer must return to 0.
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Limite atingido!");
      lcd.setCursor(0,1);
      lcd.print("Teste parado!");
      delay(1000);
      updateMenu();
      Pot.set(0);
      reps = 0;
      return;
    }
     
  }

}

void manual()
{
  // In this function the inputs in the joystick translate directly into current at the coil.
  // Because of that it was deemed necessary to remind the operator to not exceed the limits
  // of the engine in step 1.
  // The function works by collectin data using the "Collect_Data()" function and checking in
  // each loop if the up or down button of the joystick has been pressed. If that is true,
  // then the current is incremented or decremented, respectively. If the middle button is 
  // pressed the function is terminated. For that "level" and "step" must be set to 1 and 0 and
  // the digital potentiometer must be set to 0.  
  if (step == 0)
  {
    lcd.clear();
    lcd.setCursor(4,0);
    lcd.print("Cuidado!");
    lcd.setCursor(0,1);
    lcd.print("Nao ha limite!");
      
    delay(2000);
    step = 1;
  }
  else if (step == 1)
  {
    strcpy(type,"manual");
    Collect_Data();

    int upState = digitalRead(upPin);
    int downState = digitalRead(downPin);

    // Increment potentiometer value if the up button is pressed
    if (upState == LOW) {
      decreasePot(1);         // Increment
      delay(50);  // Small delay for debouncing
    }

    // Decrement potentiometer value if the down button is pressed
    if (downState == LOW) {
      increasePot(1);   // Decrement
      delay(50);  // Small delay for debouncing
    }

    buttonState = digitalRead(selectPin); // Read the state of the button

    if (buttonState == LOW) { // Button is pressed (assuming active-low configuration)
      if (!buttonPressed) { // Button was not previously pressed
        buttonPressStartTime = millis(); // Record the time when the button was first pressed
        buttonPressed = true; // Update button state to pressed
        buttonPressedFor5Seconds = false; // Reset flag for press duration
      } else {
        // Check if the button has been pressed for 5 seconds
        if ((millis() - buttonPressStartTime) >= pressDuration) {
          buttonPressedFor5Seconds = true; // Set flag indicating button has been pressed for 5 seconds
        }
      }
    } else { // Button is released
      if (buttonPressed) { // Button was previously pressed
        buttonPressed = false; // Update button state to released
        if (buttonPressedFor5Seconds) {
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("Voltar ao menu!");
          //Serial.println("A voltar ao menu principal");
          level = 1;
          step = 0;
          buttonPressedFor5Seconds = false; // Reset flag
          Pot.set(0);  
          reps = 0;
          delay(1500);
          updateMenu();
        }
      }
    }
  }

}

void setWiper(int value) {
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(0x00);  // Command byte for wiper 0
    SPI.transfer(value & 0x7F);  // 7-bit wiper value
    digitalWrite(CS_PIN, HIGH);
}

void increasePot(int step) {
    wiperValue += step;
    if (wiperValue > 127) wiperValue = 127;
    setWiper(wiperValue);
}

void decreasePot(int step) {
    wiperValue -= step;
    if (wiperValue < 0) wiperValue = 0;
    setWiper(wiperValue);
}

void current()
{
  // function to display the value of the current during the selection process
    
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Corrente inicial");
  lcd.setCursor(0,1);
  lcd.print(initial_current);
  lcd.print(" A");
}

void rpm_selecter() {
  // function to display the value of the minimum rpm during the selection process
    
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("RPM minimas");
  lcd.setCursor(0,1);
  rpm_adjusted = round(rpm_min/50)*50;        // this step is done to round the value of rpm to the nearest 50
  lcd.print(rpm_adjusted);
}

void countRpm() {
  Duration = micros() - PrevTime;                       // Calculates time difference between revs in microsecond
  PrevTime = micros();
}

void RPM()
{
  rpm = 60000000 / Duration;                                 // rpm = (1/ time millis)*1000*1000*60;
  if (micros() - PrevTime  > 2*1000000)                       // Check if motor stopped - unchanged after 2s
  {
    rpm = 0;
  }
}

void Collect_Data()
{
  time_now = seconds() - time_abs/1000.0;                          // time since the start of the first test
  // Load Cell Code
  // Read the weight
  Mass = scale_engine.get_units(3)/4;                              // Average of 3 readings from the test bench load cell 
                                                                   // it also takes into account the 4:1 pulley system implemented
  Weight = Mass * 1e-3 * 9.80665;                                  // Conversion of mass into weight
  Torque = Weight * Distance;

  fuel_mass = scale_fuel.get_units(3);                             // Average of 3 readings from the fuel load cell  
      
  current_load = ina219_coil.getShuntVoltage_mV()*50/75;            // Read the voltage output from the coil 
                                                                    // shunt and calculate the current on the circuit

  current_load2 = ina219_motor.getShuntVoltage_mV()*50/75;          // Read the voltage output from the motor shunt
                                                                    // and calculate the current on the circuit

  real_rpm = RPM()*4;                                               // Calculate the engine shaft RPM 

  Total_Power = Torque * real_rpm;

  if (reps == 0)
  {
    // Prints a header if it is the first rep of testing with is column data and unit.  
    Serial.println("type,Load current,Duration,RPM,Mass [g],Weight [N],Torque[Nm],Total Power [W],Current Consumed by motor[A],Fuel mass [g]");
    reps = 1;
  }


  // Print all data into the serial monitor. To record this any recording software can be used.
  // Data is printed in order to be recorded as a .csv for easier handling.  
  Serial.print(type);
  Serial.print(",");
  Serial.print(current_load);
  Serial.print(",");
  Serial.print(time_now,4);
  Serial.print(",");
  Serial.print(rpm);
  Serial.print(",");
  Serial.print(Mass);
  Serial.print(",");
  Serial.print(Weight);
  Serial.print(",");
  Serial.print(Torque);
  Serial.print(",");
  Serial.print(Total_Power);
  Serial.print(",");
  Serial.print(current_load2);
  Serial.print(",");
  Serial.println(fuel_mass);  

  if (step == 5 || step == 1){
    // To ensure that all important information is displayed in the LCD during steps 6 and 7 this
    // this conditional statement is implemented. 
      
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("RPM:");
    lcd.print(rpm);
    lcd.setCursor(0,1);
    lcd.print("Torque:");
    lcd.print(Torque);
  }
}
