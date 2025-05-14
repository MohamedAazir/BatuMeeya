#include <SoftwareSerial.h>

SoftwareSerial mySerial(A3, A2); // RX, TX

void setup() {
  Serial.begin(115200);   // Start the hardware serial
  mySerial.begin(115200); // Start the software serial on A3 (RX), A2 (TX)
}

void loop() {
  if (mySerial.available()) {
    // Read the incoming data
    String data = mySerial.readStringUntil('\n');
    
    // Print the data to the Serial Monitor
    Serial.println(data);
    delay(1000);
  }
}
