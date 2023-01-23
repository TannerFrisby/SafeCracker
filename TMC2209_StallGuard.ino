/**
 * Libraries:
 *  TMCStepper by teemuatlut version 0.7.3
 *  AccelStepper by Mike McCauley version 1.64.0
 * 
 * Hardware: 
 *  DOIT ESP32 DEVKIT V1
 *  BTT TMC2209-V1.2
 *  Generic 4-wire NEMA 23 stepper motor
 * 
 * Purpose:
 *  Tune the stall sensitivity of a given hardware configuration
 * 
 * Summary:
 *  Initialize the TMC2209 over UART
 *  Rotate between +-(distance) steps while monitoring for stall
 *  Upon a stall:
 *    Print "Stalled" to the serial monitor
 *    Decelerate to a stop
 *    Delay
 *    Print "Resumed" to the serial monitor
 *    Proceed to the pre-stall target location
 */

// Libraries
#include <TMCStepper.h>
#include <AccelStepper.h>

// Pin definitions             Wiring ("-" is a wire)
#define EN_PIN           5  // ESP32 - TMC2209 Enable
#define DIR_PIN          15 // ESP32 - TMC2209 Direction
#define STEP_PIN         2  // ESP32 - TMC2209 Step
#define STALL_PIN        4  // ESP32 - TMC2209 DIAG
#define SERIAL_PORT Serial2 // ESP32 TX - 1k ohm - ESP32 RX - TMC2209 PDN 

// Setting up drivers
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2, 0b00 when left floating
// higher value of STALL_VALUE increases stall sensitivity
// diag pin pulsed HIGH when SG_RESULT falls below 2*STALL_VALUE
// must be in StealthChop Mode for stallguard to work
// Value of TCOOLTHRS must be greater t
#define STALL_VALUE     50 // [0..255]
#define R_SENSE 0.11f // Match to your driver, BTT TMC2209-V1.2 appears to be 0.11f
TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Stepper motion settings
float peakSpeed = 4000.0 ;        // Steps per second
float peakAccel = 10000.0 ;       // Steps per second^2
long distance = 5000 ;            // Steps +- starting location

// Stall detection
volatile bool stalled = false ;   // Indication of DIAG pulsing HIGH
void stallInterrupt(){
  stalled = true;                 // Flag set when motor stalls
}
#define STALL_DELAY 500           // Milliseconds to hold the motor idle

void setup() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STALL_PIN, INPUT);
  
  Serial.begin(115200);           // Serial monitor
  SERIAL_PORT.begin(115200);      // HW UART drivers

  driver.begin();                 // UART: Init SW UART (if selected) with default 115200 baudrate
  driver.toff(4);                 // Enables driver in software (some examples use 5, others use 4)
  driver.blank_time(24);          // No comment given in the example this is taken from
  driver.rms_current(600);        // Set motor RMS current
  driver.microsteps(8);           // Set microsteps
//  driver.en_spreadCycle(false);   // Toggle spreadCycle on TMC2209
  driver.pwm_autoscale(true);     // Needed for stealthChop
  driver.semin(5);                // No comment given in the example this is taken from
  driver.semax(2);                // No comment given in the example this is taken from
  driver.sedn(0b01);              // No comment given in the example this is taken from
  driver.TCOOLTHRS(0x000FF);      // This is the lower velocity threshold (units?) at which StallGuard could trigger, 20bit max
  driver.SGTHRS(STALL_VALUE);     // STALL_VALUE is set above
  attachInterrupt(digitalPinToInterrupt(STALL_PIN), stallInterrupt, RISING);
  digitalWrite(EN_PIN, LOW);      // Enable driver in hardware

  // Setup stepper parameters
  stepper1.setMaxSpeed(peakSpeed);      // peakSpeed is set above
  stepper1.setAcceleration(peakAccel);  // peakAccel is set above
  stepper1.moveTo(distance);            // distance is set above, motion doesn't occur until stepper1.run() is called
}


void loop() {
  // Drive the stepper motor
  stepper1.run();                     // Pulse step if it is due for a pulsing.

  // Change the stepper motor's direction
  if (stepper1.distanceToGo() == 0){  // If the stepper has reached its target angular displacement
    distance = -distance;             // Invert "distance" (oscilate between +- "distance")
    stepper1.moveTo(distance);        // Set the new target location
  }
  
  // Handle a stall event
  if (stalled){                           // If the stall flag is set
    stalled = false ;                     // Reset the stall flag
    Serial.println("Stalled");            // Serial message
    stepper1.stop();                      // Decelerate the stepper to a stop
    while(stepper1.distanceToGo() > 0){   // While the stepper is halting
      stepper1.run();                     // Pulse as required
    }
    delay(STALL_DELAY);                   // Idle the stepper motor
    Serial.println("Resuming");           // Serial message
    stepper1.moveTo(distance);            // Reset the target to the pre-stall location
  }
}
