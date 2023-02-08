#include <Servo.h>
#define sensor A0
Servo myservo;
//Definicao de variaveis
double uK, infraVermelho, distance_cm, tempo;
double xss1 = 0, xss2 = 0, uss = 0;

//Referência que queremos que o sistema se estabilize
double distRefCm = 9;

double xhatc1 = 0, xhatc2 = 0, xp1 = 0, xp2 = 0;
double MatC1 = 1, MatC2 = 0;
double volts;

//Ganhos do observador e do regulador
double K1 = 3.2, K2 = 3.5;
double Kec1 = 0.3397, Kec2 = 0.8440;
double Kep1 = 0.3748, Kep2 = 0.8440;

//Ganhos da intro de ref
double Nx1 = 1, Nx2 = 0;
double Nu = 0;

//Parametros do modelo
double Phi11 = 1, Phi12 = 0.0416, Phi21 = 0, Phi22 = 1;
double Gamma1 = 0.0004, Gamma2 = 0.0189;

//Periodo de amostragem
double Ta = 0.005;

void setup() {
  Serial.begin(2000000);  
  myservo.attach(9);  // O servo motor está na porta 9 do arduíno
  myservo.write(90);  // O motor inicia em 90º
  delay(5000);
}

void loop() {
  tempo = micros();

  // Calculando xss e uss
  xss1 = Nx1*distRefCm;
  xss2 = Nx2*distRefCm; 
  uss = Nu*distRefCm;

  //Lendo sensor
  infraVermelho = analogRead(A0);
  volts = infraVermelho * 0.0048828125; 
  distance_cm = 32.263 * pow(2.71828182,-0.791*volts)-15.00;
    
  //Observador de estados - Correcao
  xhatc1 = xp1 + Kec1*(distance_cm - (MatC1*xp1 + MatC2*xp2));
  xhatc2 = xp2 + Kec2*(distance_cm - (MatC1*xp1 + MatC2*xp2));
    
  //Calculo da acao de controle
  uK = (-K1*(xhatc1-xss1) + (-K2*(xhatc2-xss2))) + uss;

  //Saturando controle
  uK = max(uK,-45); uK = min(uK, 90);

  //Observador de estados - Predicao
  xp1 = (Phi11*xhatc1 + Phi12*xhatc2) + Gamma1 * uK;
  xp2 = (Phi21*xhatc1 + Phi22*xhatc2) + Gamma2 * uK;
  
  //Aplicando controle na planta
  myservo.write((int)uK+90);

  //Printando alguns valores úteis
  Serial.print("   ");
  Serial.print(distance_cm);
  Serial.print("  ");
  Serial.print(xhatc1);
  Serial.print(xhatc2);
  Serial.print(" ");
  Serial.print("  ");
  Serial.print((int)uK);
  Serial.print("  ");
  Serial.println(tempo/1E6);

  //Esperando pelo prox. periodo de amostragem
  while ((micros() - tempo)/1E6 < Ta){ }//em s
}