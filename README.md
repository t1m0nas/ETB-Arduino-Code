# ETB-Arduino-Code
--------------------------------------------- Code structure and information ----------------------------------------------------

  This code was made to control and collect data from an Engine Test Bench (ETB). The ETB was developed as a master's dissertation at University of Beira Interior. The ETB will have, in this version, 2 different modes of operation, "Static" and "Manual". The static mode is automatic with just the selection of the initial load and minimum rpm needed. On the "manual" mode the test is done manually with the user having to select the load.
  The code is structured in levels on loop() and steps on the different modes functions. Every time a task is completed the step or level value is incremented until it is needed to return to a previous level/step. 
