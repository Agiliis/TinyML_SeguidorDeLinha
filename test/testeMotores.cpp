#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define PIN_LED   48  // Pino conectado ao LED RGB na maioria dos ESP32-S3 DevKitC-1
#define NUM_LEDS  1   // Quantidade de LEDs na placa
#define LO 0
#define HI 256

// Inicializa a biblioteca NeoPixel
Adafruit_NeoPixel strip(NUM_LEDS, PIN_LED, NEO_GRB + NEO_KHZ800);

// Definição dos Pinos - Motor A
const int MA_IN1 = 13;
const int MA_IN2 = 12;

// Definição dos Pinos - Motor B
const int MB_IN1 = 10;
const int MB_IN2 = 11;

// Configuração de Velocidade (0 a 255)
int velocidade = 180; 

void setup() {
  strip.begin();
  strip.setBrightness(30); // Define o brilho (0 a 255) para não ficar forte demais
  strip.show();

  pinMode(MA_IN1, OUTPUT);
  pinMode(MA_IN2, OUTPUT);
  pinMode(MB_IN1, OUTPUT);
  pinMode(MB_IN2, OUTPUT);
  
  // Serial.begin(115200);
  // Serial.println("Iniciando teste de motores em 3 segundos...");
  delay(3000);
}

void reset_motores() {
  // Estado de Standby (L, L)
  analogWrite(MA_IN1, LO);
  analogWrite(MA_IN2, LO);
  analogWrite(MB_IN1, LO);
  analogWrite(MB_IN2, LO);
}

void freio() {
  // Estado de Freio Ativo (H, H) para travar o robô
  analogWrite(MA_IN1, HI);
  analogWrite(MA_IN2, HI);
  analogWrite(MB_IN1, HI);
  analogWrite(MB_IN2, HI);
}
void mover(int vA, int vB) {
  // --- MOTOR A (D32 e D33) ---
  
  vA*=-1;
  vB*=-1;

  if (vA > 0) {
    // Frente: IN1 com PWM, IN2 em 0
    analogWrite(MA_IN2, 0); 
    analogWrite(MA_IN1, abs(vA));
  } else if (vA < 0) {
    // Trás: IN1 em 0, IN2 com PWM
    analogWrite(MA_IN1, 0);
    analogWrite(MA_IN2, abs(vA));
  } else {
    // Parar (Freio Ativo)
    analogWrite(MA_IN1, 255);
    analogWrite(MA_IN2, 255);
  }

  // --- MOTOR B (D25 e D26) ---
  if (vB > 0) {
    analogWrite(MB_IN2, 0);
    analogWrite(MB_IN1, abs(vB));
  } else if (vB < 0) {
    analogWrite(MB_IN1, 0);
    analogWrite(MB_IN2, abs(vB));
  } else {
    analogWrite(MB_IN1, 255);
    analogWrite(MB_IN2, 255);
  }
}

void loop() {
  strip.setPixelColor(0, strip.Color(0, 255, 0));
  strip.show();
  // Serial.println("Frente...");
  mover(velocidade, velocidade);
  delay(3000);

  strip.setPixelColor(0, strip.Color(255, 0, 0));
  strip.show();
  // Serial.println("Freio!");
  freio();
  delay(3000);

  strip.setPixelColor(0, strip.Color(0, 0, 0));
  strip.show();
  // Serial.println("Parado. Reiniciando ciclo...");
  reset_motores();
  delay(3000);

}