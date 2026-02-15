# Project 3: Automatic Windshield Wipers subsystem for Driver's Ed Vehicle
## Team Members: Mia Vatanapradit, Ayanat Zhumagaliyeva
### System Behavior:
This project implements a Driver’s Ed Safety Car system that enforces supervision and basic safety rules by requiring both front seats to be occupied and both seatbelts fastened before the engine can start. It displays a welcome message when the driver sits down and warning messages with an alarm if ignition is attempted without meeting the requirements, while still allowing repeated start attempts and letting the driver turn the engine off with the ignition button after the first attempt. The system is extended with a windshield wiper subsystem that only operates when the engine is running and supports HIGH, LOW, Intermittent (INT), and OFF modes, where HIGH and LOW continuously sweep the wipers between 0 and 90 degrees at different speeds, INT runs at low speed with user-selected short, medium, or long delays between wipes, and OFF (or turning off the engine) completes the current sweep and returns the wipers to the 0-degree resting position, with the LCD display showing the selected mode and delay.
### Design Alternatives:
For the windshield wiper motor simulation, we selected a position servo motor instead of a continuous servo motor because it provides direct control over the wiper angle using duty cycles, which makes it much easier to meet the requirement of sweeping between fixed positions such as 0 and 90 degrees. Position servos use PWM signals to move to precise angular positions, allowing consistent and repeatable wiper motion, while continuous servos use PWM to control speed and direction rather than position. Since the wiper system only requires a limited and well-defined range of motion rather than unlimited rotation, a continuous servo would add unnecessary complexity and functionality beyond what the design requires.
### Starting Repository:
This system was built from [Mia's repository](https://github.com/skobele28/Skobel-Vatanapradit-Project2)

### Summary of Testing results:
**Ignition Subsystem:**
| **Specification** | **Test Process** | **Results** |
|--------------|--------------|---------|
| When the driver is seated, the system is initiated and a welcome message displays. | _1 button: DS_ <br> 1. DS pressed <br> 2. DS and any other buttons (PS, DB, PB) are pressed <br>3. PS, DB, PB are pressed | _All tests passed_ <br>1. Welcome message appears on monitor<br>2. Nothing happens<br>3. Nothing happens |
| Ignition is enabled when both seats are occupied (DS, PS) and both seat belts are fastened (DB, PB). Green LED turns on if ignition is enabled successfully. | _4 buttons: DS, DB, PS, PB_<br>1. DS, PS, DB, PB are pressed simultaneously<br>2. Any combination of DS, PS, DB, PB are pressed simultaneously | _All tests passed_<br>1. Green LED turns on<br>2. Green LED stays turned off |
| Start engine (turn off green LED, turn on red LED) after ignition is enabled (green LED on) and ignition button is pressed (before button released). If ignition is not enabled, trigger a sound alarm. | _1 button: Ignition_<br>1. Ignition button is pressed when ignition enabled (green LED turned on)<br>2. Ignition button is pressed when ignition not enabled (green LED not turned on) | _All tests passed_<br>1. Green LED turns off, red LED turns on, LCD displays corresponding message<br>2. Sound alarm triggered, serial monitor displays errors (DS, PS not occupied, DB, PB not fastened – in any combination) |
| The engine stays running regardless of the status of seats and seatbelts. Indication: red LED stays on. | _4 buttons: DS, DB, PS, PB_ <br>1. DS, PS, DB, PB are released | _Test passed_ <br>1. Red LED stays turned on, LCD display message stays on |
| Turn engine off after previously successfully starting engine | _1 button: Ignition_ <br>1. Ignition button is pressed while the engine is running | _Test passed_ <br>1. Red LED turns off, LCD display message changes to a new corresponding message |
| Reattempt starting engine after successfully turning it off | Repeat test cases above after successfully turning engine off | _Test passed_ <br>1. Results match their specifications |
**Window Wiper Subsystem:**
| **Specification** | **Test Process** | **Results** |
|--------------|--------------|---------|
| Specification | Test Process | Results |
| Specification | Test Process | Results |
| Specification | Test Process | Results |
| Specification | Test Process | Results |
| Specification | Test Process | Results |
