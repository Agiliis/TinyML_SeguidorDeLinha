#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define PIN_LED   48  // Pino conectado ao LED RGB na maioria dos ESP32-S3 DevKitC-1
#define NUM_LEDS  1   // Quantidade de LEDs na placa
#define LO 0
#define HI 256

// Inicializa a biblioteca NeoPixel
Adafruit_NeoPixel strip(NUM_LEDS, PIN_LED, NEO_GRB + NEO_KHZ800);

const int G1 = 14;
const int G2 = 4;

void setup() {
  strip.begin();
  strip.setBrightness(30); // Define o brilho (0 a 255) para não ficar forte demais
  strip.show();

  pinMode(G1, INPUT);
  pinMode(G2, INPUT);
  
  Serial.begin(115200);
  // Serial.println("Iniciando teste de motores em 3 segundos...");
}

void loop() {
  strip.setPixelColor(0, strip.Color(0, 255, 0));
  strip.show();
  // Serial.println("Frente...");

  Serial.println("G1:" + String(digitalRead(G1)) + " G2:" + String(digitalRead(G2)));
}