int sensorPin = A0;    // select the input pin for the potentiometer
int ledPin = 10;      // select the pin for the LED
int ledVal = 0;
int sensorValue = 0;  // variable to store the value coming from the sensor
int values = 0;
int valuesTemp = 0;

void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);  
  pinMode(sensorPin, INPUT);
  Serial.begin(9600); // Open serial monitor at 115200 baud to see ping results.
  delay(1000);
}

void loop() {
    for (int count = 0; count < 400; count++) { 
      sensorValue = analogRead(sensorPin);
//        sensorValue = digitalRead(sensorPin);
      if(sensorValue > 500) {
//        if(sensorValue > 0) {
          valuesTemp++;
        }
    }
    Serial.println(valuesTemp);
    if(valuesTemp > 100) {
    Serial.println("-----------------");
      if(ledVal == 1) {
        Serial.println("OFF");
         ledVal = 0; 
         digitalWrite(ledPin,0);
         delay(300);
      }
      else {
        Serial.println("ON");
         ledVal = 1; 
         digitalWrite(ledPin,1);
         delay(300);
      }
    }
    valuesTemp = 0;
    delay(50);
}
