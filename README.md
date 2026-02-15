# Project 3: Automatic Windshield Wipers subsystem for Driver's Ed Vehicle
## Team Members: Mia Vatanapradit, Ayanat Zhumagaliyeva
### System Behavior:
This project implements a Driver’s Ed Safety Car system that enforces supervision and basic safety rules by requiring both front seats to be occupied and both seatbelts fastened before the engine can start. It displays a welcome message when the driver sits down and warning messages with an alarm if ignition is attempted without meeting the requirements, while still allowing repeated start attempts and letting the driver turn the engine off with the ignition button after the first attempt. The system is extended with a windshield wiper subsystem that only operates when the engine is running and supports HIGH, LOW, Intermittent (INT), and OFF modes, where HIGH and LOW continuously sweep the wipers between 0 and 90 degrees at different speeds, INT runs at low speed with user-selected short, medium, or long delays between wipes, and OFF (or turning off the engine) completes the current sweep and returns the wipers to the 0-degree resting position, with the LCD display showing the selected mode and delay.
### Design Alternatives:
For the windshield wiper motor simulation, we selected a position servo motor instead of a continuous servo motor because it provides direct control over the wiper angle using duty cycles, which makes it much easier to meet the requirement of sweeping between fixed positions such as 0 and 90 degrees. Position servos use PWM signals to move to precise angular positions, allowing consistent and repeatable wiper motion, while continuous servos use PWM to control speed and direction rather than position. Since the wiper system only requires a limited and well-defined range of motion rather than unlimited rotation, a continuous servo would add unnecessary complexity and functionality beyond what the design requires.
### Starting Repository:
This system was built from [Mia's repository](https://github.com/skobele28/Skobel-Vatanapradit-Project2)

### Summary of Testing results:
//to add
