// The is the code for the ESP32 microcontroller to communicate through a bluetooth terminal.
// It is used to unlock the gate based on the serial input from the safety gate controller.
// Currently this needs to be uploaded to the ESP32 microcontroller via the Arduino IDE, ran with a windows machine.

// Author: Max Chen
// Date: 2025-02-13
// v0.1.1

// This is the code for the ESP32 microcontroller to communicate through a bluetooth terminal.
#include <BluetoothSerial.h>

BluetoothSerial ESP_BT;
const int ledPin = 2;  // Onboard LED pin (usually pin 2 on ESP32)
const int greenLEDPin = 16;  // Green LED pin for Unlocked state
const int redLEDPin = 17;    // Red LED pin for Locked state

// Define states
enum State { LOCKED, UNLOCKED };
State currentState = UNLOCKED;  // Initial state

void setup() 
{
  Serial.begin(115200);  // Start serial communication at 115200 baud
  pinMode(ledPin, OUTPUT);
  pinMode(greenLEDPin, OUTPUT);  // Set green LED pin as output
  pinMode(redLEDPin, OUTPUT);    // Set red LED pin as output
  ESP_BT.begin("I_AM_A_GATE-ESP32");  // Start Bluetooth with a name
  Serial.println("Bluetooth device is ready to pair.");
  Serial.println("Enter 'send' to send a message over Bluetooth.");
}

void loop() 
{
  // Check if data is available in the serial buffer
  if (Serial.available())  
  {
    String input = Serial.readStringUntil('\n');  // Read the serial input until a newline character
    
    // Trim any leading/trailing whitespace and convert to lowercase
    input.trim();
    
    // Check if the input matches the word "unlock"
    if (input == "unlock" && currentState != UNLOCKED) 
    {
      // Send Bluetooth message
      ESP_BT.println("OPEN THE GATE!");
      Serial.println("Message sent!");
      currentState = UNLOCKED;  // Update state

      // Blink the LED after sending the message
      dotdotdot();
      digitalWrite(greenLEDPin, HIGH);  // Turn on green LED
      digitalWrite(redLEDPin, LOW);      // Turn off red LED
    }

    // Check if the input matches the word "lock"
    if (input == "lock" && currentState != LOCKED) 
    {
      // Send Bluetooth message
      ESP_BT.println("LOCK THE GATE!");
      Serial.println("Message sent!");
      currentState = LOCKED;  // Update state

      // Blink the LED after sending the message
      dashDot();
      digitalWrite(greenLEDPin, LOW);   // Turn off green LED
      digitalWrite(redLEDPin, HIGH);     // Turn on red LED
    }
  }
}

// Blink the Onboard LED for -.
void dashDot()
{
  digitalWrite(ledPin, HIGH);  // Turn LED on
  delay(175);                   // Wait for 175ms
  digitalWrite(ledPin, LOW);    // Turn LED off
  delay(75);                   // Wait for 75ms
  digitalWrite(ledPin, HIGH);  // Turn LED on
  delay(75);                   // Wait for 75ms
  digitalWrite(ledPin, LOW);    // Turn LED off
  delay(75); 
}

// Blink the Onboard LED for ...
void dotdotdot() 
{
  for (int i = 0; i <= 3; i++) 
  {
    digitalWrite(ledPin, HIGH);  // Turn LED on
    delay(75);                   // Wait for 75ms
    digitalWrite(ledPin, LOW);   // Turn LED off
    delay(75);                   // Wait for 75ms
  }
}
