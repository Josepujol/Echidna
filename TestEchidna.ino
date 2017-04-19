

// PULSADORES
const int buttonRPin = 2;
const int buttonLPin = 3;
int buttonLState = 0;
int buttonRState = 0;

// RGB
const int rgbledRPin = 5;
const int rgbledGPin = 9;
const int rgbledBPin = 6;

// LEDS

const int ledRPin = 13;
const int ledGPin = 11;
const int ledOPin = 12;

// Zumbador
int buzzerPin = 10;

//LDR
const int LDRPin = A5;
int valorLDRini;
int LDRvalue = 0;

// Joystick
const int xjoyPin = A0;
const int yjoyPin = A1;
int xjoyValue = 0;
int yjoyValue = 0;

// Acelerometer
const int xacelPin = A2;
const int yacelPin = A3;
int xacelValue = 0;
int yacelValue = 0;

//Distance Sensor
const int distSensorPin = A4;
int distancia;

String zumbador;
String rgb;
String ldr;
String acelerometro;
String distance;
String lm35;

void setup() {

  // initialize the LED pin as an output:
  pinMode(ledRPin, OUTPUT);
  pinMode(ledGPin, OUTPUT);
  pinMode(ledOPin, OUTPUT);

  // inicializamos zumbador como salida digital
  pinMode(buzzerPin, OUTPUT);

  // initialize the pushbutton pin as an input:
  pinMode(buttonRPin, INPUT);
  pinMode(buttonLPin, INPUT);

  // abrimos el puerto serie
  Serial.begin(9600);

  salidas();
  pulsadores();
  sensores();

}


void salidas() {
  Serial.println("======COMPROBAMOS LAS SALIDAS======");
  Serial.println("======Probando el zumbador======");
  digitalWrite(buzzerPin, HIGH);
  delay(500);
  digitalWrite(buzzerPin, LOW);
  Serial.println("======Probamos los leds de colores======");
  Serial.println("LED VERDE ON");
  digitalWrite(ledGPin, HIGH);
  delay(2000);
  digitalWrite(ledGPin, LOW);
  Serial.println("LED NARANJA ON");
  digitalWrite(ledOPin, HIGH);
  delay(2000);
  digitalWrite(ledOPin, LOW);
  Serial.println("LED ROJO ON");
  digitalWrite(ledRPin, HIGH);
  delay(2000);
  digitalWrite(ledRPin, LOW);

  Serial.println("======Probamos EL LED RGB======");
  Serial.println("LED RGB VERDE ON");
  analogWrite(rgbledGPin, 255);
  delay(2000);
  analogWrite(rgbledGPin, 0);
  Serial.println("LED RGB AZUL ON");
  analogWrite(rgbledBPin, 255);
  delay(2000);
  analogWrite(rgbledBPin, 0);
  Serial.println("LED RGB ROJO ON");
  analogWrite(rgbledRPin, 255);
  delay(2000);
  analogWrite(rgbledRPin, 0);
}

void pulsadores() {
  Serial.println("======COMPROBAMOS LOS PULSADORES======");
  Serial.println("Accionea el selector al Modo Sensores");
  Serial.println("Presiona el pulsador derecho");
  while (buttonRState == LOW) {
    buttonRState = digitalRead(buttonRPin);
  }
  Serial.println("El pulsador derecho funciona correctamente");
  Serial.println("Presiona el pulsador izquierdo");
  while (buttonLState == LOW) {
    buttonLState = digitalRead(buttonLPin);
  }
  Serial.println("El pulsador izquierdo funciona correctamente");
}

void sensores() {

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.println("======Probando la LDR======");
  valorLDRini = analogRead(LDRPin);
  Serial.println("Tape el sensor LDR con la mano");
  LDRvalue = valorLDRini;
  Serial.println("Si no se detecta cambio en el sensor, pulse el boton izquierdo");
  buttonLState = digitalRead(buttonLPin);
  while (valorLDRini - LDRvalue < 100 && buttonLState == LOW) {
    LDRvalue = analogRead(LDRPin);
  }
  if (buttonLState == HIGH) {
    ldr = "#El LDR no funciona correctamente";
  }
  else {
    ldr = "#El LDR funciona correctamente";
  }
  while (buttonLState == HIGH) {
    buttonLState = digitalRead(buttonLPin);
  }

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.println("======Probando JOYSTICK======");
  Serial.println("Eje X Joystick");
  Serial.println("Mueve el Eje X del Joystick a la izquierda");
  while (xjoyValue > 100) {
    xjoyValue = analogRead(xjoyPin);
  }
  Serial.println("Mueve el Eje X del Joystick a la derecha");
  while (xjoyValue < 900) {
    xjoyValue = analogRead(xjoyPin);
  }
  Serial.println("El Joystick X funciona bien");
  ///////////////////////////////////////////
  Serial.println("Eje Y Joystick");
  Serial.println("Mueve el Eje Y del Joystick arriba");
  while (yjoyValue < 900) {
    yjoyValue = analogRead(yjoyPin);
  }
  Serial.println("Mueve el Eje Y del Joystick abajo");
  while (yjoyValue > 100) {
    yjoyValue = analogRead(yjoyPin);
  }
  Serial.println("El Joystick y funciona bien");

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.println("======Probando ACELERÓMETRO======");
  Serial.println("Eje X Acelerómetro");
  Serial.println("Inclina el escudo hacia la izquierda");
  while (xjoyValue > 350) {
    xacelValue = analogRead(xacelPin);
  }
  Serial.println("Inclina el escudo hacia la derecha");
  while (xjoyValue < 700) {
    xacelValue = analogRead(xacelPin);
  }
  Serial.println("El Acelerómetro funciona bien");
  ///////////////////////////////////////////
  Serial.println("Eje Y Acelerómetro");
  Serial.println("Inclina el escudo hacia la delante");
  while (yjoyValue > 350) {
    yacelValue = analogRead(yacelPin);
  }
  Serial.println("Inclina el escudo hacia la atras");
  while (yjoyValue < 700) {
    yacelValue = analogRead(yacelPin);
  }
  Serial.println("El Acelerómetro funciona bien");

}


void loop() {
}


