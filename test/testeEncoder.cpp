#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define PIN_LED   48  // Pino conectado ao LED RGB na maioria dos ESP32-S3 DevKitC-1
#define NUM_LEDS  1   // Quantidade de LEDs na placa
#define LO 0
#define HI 256

// Inicializa a biblioteca NeoPixel
Adafruit_NeoPixel strip(NUM_LEDS, PIN_LED, NEO_GRB + NEO_KHZ800);

// --- Configurações do Encoder ---
const int ENCODER1 = 4; // Conecte ao pino D0 (Digital Out) do encoder
const int ENCODER2 = 14; // Conecte ao pino D0 (Digital Out) do encoder
const int FUROS_DISCO = 21;  // Quantidade de furos no disco do seu motor

// Variáveis para a interrupção e cálculo de tempo
// A palavra 'volatile' é obrigatória para variáveis alteradas dentro de uma interrupção
volatile unsigned int contadorPulsos[2]{}; 
unsigned long tempoAnterior = 0;
const unsigned long INTERVALO_LEITURA = 1000; // Tempo em milissegundos (1 segundo)
const unsigned long FATOR_DE_CONVERSAO = 60e3 / INTERVALO_LEITURA; 

// --- Função de Interrupção (ISR) ---
// No ESP32, funções de interrupção PRECISAM do prefixo IRAM_ATTR para rodarem na memória RAM (mais rápido)
void IRAM_ATTR contarPulso1() {
  contadorPulsos[0]++;
}

void IRAM_ATTR contarPulso2() {
  contadorPulsos[1]++;
}

void setup() {
  Serial.begin(115200);
  
  // Configura o pino do encoder
  // Usamos INPUT_PULLUP caso o módulo não tenha um resistor pull-up próprio
  pinMode(ENCODER1, INPUT); 
  pinMode(ENCODER2, INPUT); 

  // Ativa a interrupção. 
  // RISING significa que ele conta 1 pulso sempre que o sinal vai de LOW para HIGH.
  attachInterrupt(digitalPinToInterrupt(ENCODER1), contarPulso1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2), contarPulso2, RISING);
  
  Serial.println("Pronto para ler RPM...");
}

void loop() {
  unsigned long tempoAtual = millis();

  // Verifica se já passou 1 segundo (INTERVALO_LEITURA)
  if (tempoAtual - tempoAnterior >= INTERVALO_LEITURA) {
    
    // Pausa as interrupções rapidamente para copiar o valor de forma segura
    noInterrupts();
    unsigned int pulsosLidos[2] = {contadorPulsos[0], contadorPulsos[1]};
    contadorPulsos[0] = contadorPulsos[1] = 0; // Zera para a próxima rodada
    interrupts(); // Retoma as interrupções

    // --- Matemática do RPM ---
    // 1. Rotações por segundo (RPS) = Pulsos lidos / Quantidade de furos
    // 2. Rotações por minuto (RPM) = RPS * 60
    float rotacoes[2], rpm[2];
    
    rotacoes[0] = (float)pulsosLidos[0] / FUROS_DISCO;
    rotacoes[1] = (float)pulsosLidos[1] / FUROS_DISCO;
    rpm[0] = rotacoes[0] * FATOR_DE_CONVERSAO;
    rpm[1] = rotacoes[1] * FATOR_DE_CONVERSAO;

    // Exibe no Monitor Serial
    for(int i=0; i<2; i++) {
      Serial.print("Sensor " + String(i));
      Serial.print(" | Pulsos no segundo: ");
      Serial.print(pulsosLidos[i]);
      Serial.print(" | Velocidade: ");
      Serial.print(rpm[i]);
      Serial.println(" RPM");
    }
    
    // Atualiza o tempo para o próximo ciclo
    tempoAnterior = tempoAtual;
  }

  // Aqui você pode colocar o restante do seu código (LED RGB, Ponte H, etc.)
  // A contagem de pulsos acontece em segundo plano, sem atrapalhar o loop!
}