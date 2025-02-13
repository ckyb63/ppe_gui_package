// The is the code for the ESP32 microcontroller to communicate through a bluetooth terminal.
// It is used to unlock the gate based on the serial input from the safety gate controller.

// Author: Max Chen
// Date: 2025-02-12
// v0.1.0

#include <BluetoothSerial.h>

BluetoothSerial ESP_BT;
const int ledPin = 2;  // Onboard LED pin (usually pin 2 on ESP32)

void setup() {
  Serial.begin(115200);  // Start serial communication at 115200 baud
  pinMode(ledPin, OUTPUT);
  ESP_BT.begin("ESP32_TISA_GATEEE");  // Start Bluetooth with a name
  Serial.println("Bluetooth device is ready to pair.");
  Serial.println("Enter 'send' to send a message over Bluetooth.");
}

void loop() {
  // Check if data is available in the serial buffer
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // Read the serial input until a newline character
    
    // Trim any leading/trailing whitespace and convert to lowercase
    input.trim();
    
    // Check if the input matches the word "send"
    if (input == "unlock") {
      // Send Bluetooth message
      ESP_BT.println("OPEN THE GATE!");
      Serial.println("Message sent!");

      // Blink the LED after sending the message
      blinkLED();
    }

        // Check if the input matches the word "send"
    if (input == "lock") {
      // Send Bluetooth message
      ESP_BT.println("LOCK THE GATE!");
      Serial.println("Message sent!");

      // Blink the LED after sending the message
      blinkLED_LONG();
    }
  }
}

void blinkLED_LONG(){
  digitalWrite(ledPin, HIGH);  // Turn LED on
  delay(175);                   // Wait for 100ms
  digitalWrite(ledPin, LOW);    // Turn LED off
  delay(75);                   // Wait for 100ms
  digitalWrite(ledPin, HIGH);  // Turn LED on
  delay(75);                   // Wait for 100ms
  digitalWrite(ledPin, LOW);    // Turn LED off
  delay(75); 
}

void blinkLED() {
  digitalWrite(ledPin, HIGH);  // Turn LED on
  delay(75);                   // Wait for 100ms
  digitalWrite(ledPin, LOW);    // Turn LED off
  delay(75);                   // Wait for 100ms
  digitalWrite(ledPin, HIGH);  // Turn LED on
  delay(75);                   // Wait for 100ms
  digitalWrite(ledPin, LOW);    // Turn LED off
  delay(75);                   // Wait for 100ms  
  digitalWrite(ledPin, HIGH);  // Turn LED on
  delay(75);                   // Wait for 100ms
  digitalWrite(ledPin, LOW);    // Turn LED off
  delay(75);                   // Wait for 100ms
}
