# ETB-Arduino-Code
--------------------------------------------- Code structure and information --------------------------------------------

  This comprehensive Arduino codebase controls and collects data from an Engine Test Bench (ETB), developed as a master's dissertation at University of Beira Interior. The ETB features two primary operation modes: Static and Manual.

--------------------------------------------- System Functionality --------------------------------------------

  Operation Modes:
    - Static Mode: Automatic operation with initial load and minimum RPM selection;
    - Manual Mode: Manual operation with user-selected load;
    
  Key Features:
    - User-friendly menu system for mode selection and parameter setup;
    - Automated data collection and storage for later analysis;
    - Real-time display of crucial test parameters on LCD display;
    
  Supported Sensors and Actuators:
    - Load Cell with HX711 amplifier;
    - RPM Sensor (Hall Effect);
    - Current Sensors (x2);
    - Digital Potentiometer (X9C102) for power supply control;
    
--------------------------------------------- Code Structure and Organization --------------------------------------------

  Code Architecture: Modular design with separate files for each functional block;

  Primary Code Segments:
    - loop(): Manages overall system state and task scheduling;
  
  Mode-Specific Functions: estatico() for Static mode, manual() for Manual mode;
  
  Helper Functions: Collect_Data(), RPM(), countRpm(), etc.
  
  Progress Tracking:
    - Levels: Used in loop() to monitor system state;
    - Steps: Employed in mode-specific functions to track progress;
  
  Incremental Progression: Step/Level values are incremented upon task completion;
  
--------------------------------------------- Key System Components --------------------------------------------

Hardware:
  Microcontroller: Arduino Mega;
  Sensors:
    - Load Cell with HX711 amplifier;
    - RPM Sensor (Hall Effect);
    - Current Sensors (x2);
  Actuators:
    - Digital Potentiometer (X9C102);
  User Interface:
    - LCD Display (I2C);
    - Joystick (with buttons);
  Power Supply: Dedicated power source for the system;
  
--------------------------------------------- Primary Functions and Their Roles --------------------------------------------

  estatico(): Handles Static mode operation, including:
    - Initial load and minimum RPM selection;
    - Automated power supply adjustment;
    - Data collection and storage;

  manual(): Manages Manual mode operation, featuring:
    - User-controlled power supply via joystick;
    - Continuous data collection until manual stop;
  
  Collect_Data(): Collects and prints data to serial monitor, including:
    - Load current;
    - Duration;
    - RPM;
    - Mass;
    - Weight;
    - Torque;
    - Total power;
    - Motor current;
  
  RPM(): Calculates engine RPM from sensor readings;

  countRpm(): Interrupt service routine for RPM sensor;

--------------------------------------------- System Operation and User Guide --------------------------------------------

  Power-On and Mode Selection:
    - Power on the system;
    - Navigate to the desired operation mode using the joystick;
  
  Static Mode Operation:
    - Select the initial current and minimum RPM using the joystick;
    - The system will automatically start the test and collect data;

  Manual Mode Operation:
    - Use the joystick to directly control the power supply;
    - To stop the test, press the joystick select button for 5 seconds;
