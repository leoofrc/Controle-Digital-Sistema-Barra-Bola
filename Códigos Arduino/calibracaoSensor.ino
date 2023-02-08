//codigo utilizado para realizar a calibracao do sensor

#include <Servo.h>
#define sensor A0
Servo servo;

void setup() {
  Serial.begin(9600);
  servo.attach(9);
  servo.write(90);
}

void loop() {
  float volts = analogRead(sensor)*0.0048828125; //Valor digital para Volts
  float distancia = ((10.1985-0.42*volts)/(volts-0.3618)); // equacao para distancia
  Serial.println(distancia); 
  //Serial.println(volts); 
  delay(100);
}
