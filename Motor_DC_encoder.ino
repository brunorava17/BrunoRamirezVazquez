#define ENCA 2  // Cable amarillo va al pin 2, es la salida A
#define ENCB 3  // Cable blanco va al pin 3, es la salida B
#define PWM 5   // Del controlador al pin 5, naranja
#define IN2 6   // Del controlador al pin 6, café
#define IN1 7   // Del controlador al pin 7, violeta

int pos = 0;                // Variable global para la posición
long prevT = 0;
float eprev = 0;            // Variables de almacenamientos globales en las estimaciones finitas
float eintegral = 0; 

// Declaración anticipada de funciones
void readEncoder();
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);

void setup(){
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
}

void loop(){
  int target = 250 * sin(prevT / 1e6); // Establecer la posición objetivo

  float kp = 1;   // Constantes del PID
  float kd = 0.025;
  float ki = 0;
  
  long currT = micros();              // Tiempo diferencial. Delta t
  float deltaT = ((float)(currT - prevT)) / 1.0e6;
  prevT = currT;      
    
  int e = pos - target;               // Calcular el error
  float dedt = (e - eprev) / deltaT;  // Calcular la derivativa
  eintegral += e * deltaT;            // Calcular la integral 

  float u = kp * e + kd * dedt + ki * eintegral;  // Calcular señal de control
  float pwr = fabs(u);                // Calcular el poder del motor
  if(pwr > 255){
    pwr = 255;
  }

  int dir = (u < 0) ? -1 : 1;         // Calcular la dirección del motor
    
  setMotor(dir, (int)pwr, PWM, IN1, IN2);

  eprev = e;                          // Almacenar error anterior

  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.println();  // Añadir salto de línea para mejorar la salida en el monitor serial
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm, pwmVal);
  if(dir == 1){
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if(dir == -1){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readEncoder(){
  int b = digitalRead(ENCB);
  if(b > 0){
    pos++;     // Sentido horario 
  } else {
    pos--;     // Sentido antihorario
  }
}
