#include <Servo.h>
#define sensor A0
Servo myservo;
//Definicao de variaveis
double uK, infraVermelho, distance_cm_ant = 0, distance_cm = 0, distancia = 0, distancia_ant = 0, tempo;
double xss1 = 0, xss2 = 0, uss = 0;

//Referência que queremos que o sistema se estabilize
double distRefCm = 0;

double xhatc1 = 0, xhatc2 = 0, xp1 = 0, xp2 = 0;
double dhat = 0; 
double dp = 0;
double MatC1 = 1, MatC2 = 0;
double volts;

//Ganhos do observador e do regulador
double K1 = 3.6784, K2 = 2.2241;
//double Kep1 = 0.3492, Kep2 = 0.8210;
double Kec1 = 0.4428, Kec2 = 12.4096, Kec3 = 7.4439;

//Ganhos da intro de ref
double Nx1 = 1, Nx2 = 0;
double Nu = 0;

//Parametros do modelo
double Phi11 = 1, Phi12 = 0.005, Phi21 = 0, Phi22 = 1;
double Gamma1 = 0.00002, Gamma2 = 0.0081;

//Periodo de amostragem
double Ta = 0.005;

void setup() {
  Serial.begin(2000000);  
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(90); 
  delay(2000);
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
  
  //Filtro Passa Baixa
  distancia_ant = distancia;
  distancia = (32.263 * pow(2.71828182,-0.791*volts)-15.00)/100;
  distance_cm_ant = distance_cm;
  distance_cm = 0.9512*distance_cm_ant + 0.02439*distancia + 0.02439 * distancia_ant;
  
  //Observador de estados - Correcao
  xhatc1 = xp1 + Kec1*(distance_cm - (MatC1*xp1));
  xhatc2 = xp2 + Kec2*(distance_cm - (MatC1*xp1));
  dhat = dp + Kec3*(distance_cm - (MatC1*xp1));
    
  //Calculo da acao de controle
  uK = (-K1*(xhatc1-xss1) + (-K2*xhatc2) - dhat*0)*180/3.1415;
  
  //Saturando controle
  uK = max(uK,-45); uK = min(uK, 90);

  //Observador de estados - Predicao
  xp1 = (Phi11*xhatc1 + Phi12*xhatc2) + Gamma1 * uK + Gamma1 * dhat;
  xp2 = (Phi21*xhatc1 + Phi22*xhatc2) + Gamma2 * uK + Gamma2 * dhat;
  dp = dhat;
    
  //Aplicando controle na planta
  myservo.write((int)uK+90);

  //Printando alguns valores úteis
  Serial.print(uK);
  Serial.print("  ");
  Serial.print(xhatc1);
  Serial.print("  ");
  Serial.print(xhatc2);
  Serial.print("  ");
  Serial.print(xss1);
  Serial.print("  ");
  Serial.println(distance_cm);

  //Esperando pelo prox. T
  while ((micros() - tempo)/1E6 < Ta){ }//em s
}
