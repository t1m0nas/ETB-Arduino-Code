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
#include <DigiPotX9Cxxx.h>                                                // Library for easier use of X9c102 potentiometer
#define seconds() (millis()/1000.0)                                       

// Load Cell with HX711
const int LOADCELL_DOUT_PIN = 4;                                          // Pin connected to DOUT of HX711
const int LOADCELL_SCK_PIN = 3;                                           // Pin connected to SCK of HX711
HX711 scale;                                                              // Start link with HX711
float Calibration = 431.31;                                                // Calibration for load cell
float Distance = 0.40;                                                    // Distance from the load cell to the axle
float Mass;                                                               // Mass measured in grams
float Weight;                                                             // Weight measured in Newtons
float Torque;                                                             // Engine torque in N.m

// RPM measurements
const int rpmPin = 2;                                                     // Pin 2 connected to hall sensor signal
int rpm;                                                                  // RPM value to be displayed
float PrevTime = 0;                                                       // Time of previous magnet detection in microseconds
float Duration = 0;                                                       // Time elapsed between magnet detection in microseconds
double time = 0;                                                          // Used to count 60 seconds from the start of the test
float time_abs = 0;                                                       // Variable used to record the absolute time in the "Collect_Data()" function

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
const int numMenuItems = 3;
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
const int current_pin = A1;                                             // Pin used to read OUT of the current sensor
const int current_pin2 = A2;                                            // Pin used to read the current used by the motor
const float sensitivity = 0.066;                                        // Sensitivity of the sensor
const float vOffset = 4.820/2;                                          // Voltage offset at 0 A of current
float current_load = 0;
float current_load2 = 0;
float average_current = 0;
float sum = 0;

// Data collection
float time_now = 0;                                                     // Time at the present reading
char type[7];                                                           // String to know which type of test it was done
int reps = 0;                                                           // Variable to know if the header is printed or not

//Joystick variables
const int upPin = 53;                                                   // Pin connected to the "up" button
const int downPin = 51;                                                 // Pin connected to the "down" button
const int selectPin = 45;                                               // Pin connected to the "select" button
const int setPin = 43;                                                  // Pin connected to the "set" button
int lastTareButtonState = HIGH; // Assuming active-low configuration
const int rstPin = 41;                                                   // Pin connected to the "Reset" button

//Digital potentiometer initialization
DigiPot Pot(33,31,29);                                                     // Pins:(INC, U/D, CS)


LiquidCrystal_I2C lcd(0x27, 16, 2);

int mapToIncrements(int value, int inMin, int inMax, int outMin, int outMax, int increment) {
    int range = (outMax - outMin) / increment;
    int mappedValue = map(value, inMin, inMax, 0, range);
    return outMin + (mappedValue * increment);
}

void setup() {
  // Definition of pin modes and activation of internal resistors
  pinMode(upPin, INPUT_PULLUP);                                         // "Up" button on the joystick
  pinMode(downPin, INPUT_PULLUP);                                       // "Down" button on the joystick
  pinMode(selectPin, INPUT_PULLUP);                                     // Button used to select and confirm modes and values
  pinMode(setPin, INPUT_PULLUP);                                        // Button used to tare the scale
  pinMode(rstPin, INPUT_PULLUP);                                        // Emergency stop button
  pinMode(rpmPin, INPUT_PULLUP);                                        // pin used for the Hall sensor signal
  
  attachInterrupt(digitalPinToInterrupt(rpmPin), countRpm, RISING);     // Interrupt used to call function countRPM() everytime there is
                                                                        // signal from the Hall Sensor
  
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);                     // Start link with HX711
  scale.tare();                                                         // Tare scale to zero
  scale.set_scale(Calibration);                                         // Adjust scale to calibration factor

  Serial.begin(9600);
  
  lcd.init();                                                   // Start I2C link with LCD
  lcd.backlight();                                              // Turn on LCD backlight
  lcd.setCursor(0, 0);
  lcd.print(" The Dyno Bench ");                                // Delay routine to stabilize scale.tare
  lcd.setCursor(0, 1);
  lcd.print("   C-MAST UBI   ");
  delay(1500);
  updateMenu();

  Pot.set(0);                                                    // Set potentiometer to 0 in the beginning
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
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Still not done");
      delay(3000);
      level = 1;
      updateMenu();
    } else if (menuIndex == 2) {
      manual();
    }
  }

    
  // In each loop, the tare and emergency buttons are checked. If any of them is pressed,
  // according action is taken.
    
  int tareButtonState = digitalRead(setPin);  // Read the button on pin 43
  
  // Check for button press (high to low transition)
  if (lastTareButtonState == HIGH && tareButtonState == LOW) {
    scale.tare();  // Tare the load cell
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Load Cell Tared");
    delay(1000);  // Short delay to show feedback on the LCD
    updateMenu(); // Refresh the menu after taring
  }
  
  lastTareButtonState = tareButtonState; // Update the last state

  // Check if Emergency Stop button (pin 41) is pressed
  if (digitalRead(41) == LOW) {
    delay(50); // Debounce delay
    if (digitalRead(41) == LOW) { // Confirm press
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
      lcd.print("2 - Transiente");
      break;
    case 2:
      lcd.setCursor(0, 1);
      lcd.print("3 - Manual");
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
  //  iables, Current and RPM are selected.  
  if(step == 0){
    current();
    step = 1;
  }
  else if (step == 1)
  {
  int new_initial_current = initial_current;  // Initialize to avoid jumps
  int upState, downState, selectState;

  // Read the joystick directions and button state
  upState = digitalRead(upPin);      // Replace with actual joystick up pin
  downState = digitalRead(downPin);  // Replace with actual joystick down pin
  selectState = digitalRead(selectPin);  // Replace with actual joystick button pin

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
    sum = 0;   // Reset sum to zero before starting

    for (int i = 1; i <= 10; i++) {
      int sensorValue = analogRead(current_pin);                      // Read the current sensor pin
      float voltage = sensorValue * (5.0 / 1023.0);                   // Convert analog reading to voltage
      current_load = (voltage - vOffset) / sensitivity;               // Calculate the current

      sum += current_load;                                            // Accumulate current values
      delay(50);                                                      // Short delay between readings to smooth out noise
    }

    average_current = sum / 10;                                       // Calculate the average current

    // Adjust potentiometer based on the average current
    if (average_current - initial_current < -0.5) {                 // If average current is less than target (below threshold)
      Pot.increase(1);                                                // Increment potentiometer
    } else if (average_current - initial_current > 0.5) {           // If average current is greater than target (above threshold)
      Pot.decrease(1);                                                // Decrement potentiometer
    } else {
      step = 5;
      time = millis();
      time_abs = millis();
    }
 
  }
  else if(step == 5)
  {
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
    int load_increment = 1;
    // processo de Aumento de corrente
    Pot.increase(1);
    step = 7;
    Collect_Data();
    time = millis();
  }
  else if(step == 7)
  {
    if(millis() - time < 2000){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("A testar limite.");
      Collect_Data();
    }

    RPM();
    int rpm_test = rpm;

    Collect_Data();

    if (rpm_min < rpm_test)
    {
      step = 5;
      time = millis();
    }
    else
    {
      level = 1;
      step = 1;
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Limite atingido!");
      lcd.setCursor(0,1);
      lcd.print("Teste parado!");
      delay(1000);
      //Serial.println("Atingido limite mínimo, teste parado!");
      updateMenu();
      Pot.set(0);
      reps = 0;
      return;
    }
     
  }

}

void manual()
{
  if (step == 0)
  {
    lcd.clear();
    lcd.setCursor(4,0);
    lcd.print("Cuidado!");
    lcd.setCursor(0,1);
    lcd.print("Nao ha limite!");
    //Serial.println("Atenção, neste teste não há limite!");
    //Serial.println("Cuidado para não danificar o motor!");
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
      Pot.increase(1);  // Increment
      delay(50);  // Small delay for debouncing
    }

    // Decrement potentiometer value if the down button is pressed
    if (downState == LOW) {
      Pot.decrease(1);  // Decrement
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
          step = 1;
          buttonPressedFor5Seconds = false; // Reset flag
          reps = 0;
          delay(1500);
          updateMenu();
        }
      }
    }
  }

}

void current()
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Corrente inicial");
  lcd.setCursor(0,1);
  lcd.print(initial_current);
  lcd.print(" A");
}

void rpm_selecter() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("RPM minimas");
  lcd.setCursor(0,1);
  rpm_adjusted = round(rpm_min/50)*50;
  lcd.print(rpm_adjusted);
  //Serial.print("RPM mínimas = ");
  //Serial.println(rpm_adjusted);
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
  time_now = seconds() - time_abs/1000.0;
  // Load Cell Code
  // Read the weight
  Mass = scale.get_units(3); // Average of 10 readings
  Weight = Mass * 1e-3 * 9.80665;
  Torque = Weight * Distance; 
      
  int sensorValue = analogRead(current_pin);                        // Reads the OUT pin from the current sensor
  float voltage = sensorValue * (4.820 / 1023.0);                     // Convert the analog reading to voltage
  current_load = (voltage - vOffset) / sensitivity;                // Calculate the current from the power supply

  int sensorValue2 = analogRead(current_pin2);                       //Reads the out pin from the current sensor
  float voltage2 = sensorValue2 * (4.820/1023);
  current_load2 = (voltage2 - vOffset) /sensitivity;

  RPM();                                                             // Calculate the engine RPM 

  Total_Power = Torque * rpm;

  if (reps == 0)
  {
    Serial.println("type,Load current,Duration,RPM,Mass [g],Weight [N],Torque[Nm],Total Power [W],Current Consumed by motor[A]");
    reps = 1;
  }


  // Print the RPM and weight on the Serial Monitor
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
  Serial.println(current_load2);

  if (step == 5 || step == 1){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("RPM:");
    lcd.print(rpm);
    lcd.setCursor(0,1);
    lcd.print("Torque:");
    lcd.print(Torque);
  }
}
