/*
  coleta_seguidor.ino
  Robô seguidor de linha com gravação de dados para treinamento.
  Placa: Arduino Nano 33 BLE Sense (nRF52840). Lógica de 3,3 V: os pinos NÃO toleram 5 V.

  Cada amostra (a cada SAMPLE_PERIOD_MS) grava:
    run            número da corrida (incrementa a cada início)
    t_ms           tempo desde o início da corrida
    s0..s7         leitura bruta do sensor de linha (s0 = sensor mais à esquerda)
    rpm_l, rpm_r   rpm das rodas, calculado a partir dos pulsos dos encoders
    pwm_l, pwm_r   comando enviado aos motores nesta amostra (0 a 255)
  O pwm da amostra k é calculado a partir de s0..s7 da mesma amostra k.
  O rpm da amostra k é a velocidade média na janela que termina em k.
  O encoder não informa sentido; como o controle só anda para frente, rpm >= 0.

  Comandos pela Serial USB:
    r  arma a gravação: após START_DELAY_MS o robô começa a seguir a linha e gravar
    s  para a gravação e os motores
    d  descarrega o buffer em CSV (termina com a linha #FIM)
    c  apaga o buffer
    v  liga/desliga o monitor (sensores e rpm a cada 200 ms, para testar na bancada)
    ?  status
  Botão em BUTTON_PIN (entre o pino e GND): alterna iniciar/parar.

  A corrida para sozinha se a linha sumir por LOST_TIMEOUT_MS ou se o buffer encher.
  Os dados ficam na RAM: desligar ou resetar a placa apaga tudo. Descarregue antes.
*/

#include <Arduino.h>
#define LO_ANALOG 0
#define HI_ANALOG 255

// ============================ CONFIGURAÇÃO ============================

// Sensor de linha digital de 8 canais. Índice 0 = sensor mais à esquerda.
// A4 e A5 têm pull-up de I2C na placa; com saída digital do módulo funciona normalmente.
const uint8_t LINE_PINS[8] = {5, 6, 7, 15, 16, 17, 18, 8};

// Nível que o módulo entrega com o sensor SOBRE a linha.
// Módulos com TCRT5000 + comparador costumam dar HIGH sobre a linha preta.
// Confirme com o comando 'v' e troque para LOW se for o contrário.
const int LINE_ON_LEVEL = HIGH;

// Encoders ópticos (disco furado + sensor de fenda), um por roda
const uint8_t  ENC_L_PIN = 4;
const uint8_t  ENC_R_PIN = 14;
const float    ENC_PPR   = 20.0f;           // número de furos do disco (pulsos por volta)
const uint32_t ENC_MIN_INTERVAL_US = 1000;  // ignora pulsos mais próximos que isso (ruído na borda do furo)
const uint32_t ENC_TIMEOUT_US = 300000;     // sem pulso por esse tempo: rpm = 0

// Ponte H com PWM + IN1 + IN2 por motor (L298N, TB6612 e similares)
struct Motor { uint8_t in1, in2; bool invert; };
const Motor MOT_L = {13, 12, true};    // invert = true se a roda girar ao contrário
const Motor MOT_R = {11, 10, false};

// Botão de iniciar/parar, ligado entre o pino e GND
const uint8_t BUTTON_PIN = 12;

// Amostragem e controle
const uint32_t SAMPLE_PERIOD_MS = 20;    // 50 Hz
const uint32_t START_DELAY_MS   = 1000;  // tempo para tirar a mão do robô
const int      BASE_PWM   = 120;         // velocidade na reta
const float    KP         = 35.0f;       // ganho proporcional (PWM por posição de sensor)
const float    KD         = 80.0f;       // ganho derivativo
const int      SEARCH_PWM = 110;         // velocidade girando para reencontrar a linha
const uint32_t LOST_TIMEOUT_MS = 1500;   // linha perdida por mais que isso: encerra a corrida

// Buffer na RAM: 16 bytes por amostra. 6000 amostras = ~96 KB = 2 min a 50 Hz
const uint32_t MAX_SAMPLES = 6000;

// ============================== ESTADO ==============================

struct Sample {
  uint32_t t_ms;
  uint16_t rpm_l_x10;   // rpm * 10
  uint16_t rpm_r_x10;
  int16_t  pwm_l;
  int16_t  pwm_r;
  uint8_t  sensors;     // bit i = leitura bruta do sensor i
  uint8_t  run;
};

Sample   samples[MAX_SAMPLES];
uint32_t nSamples = 0;

enum State { IDLE, ARMED, RECORDING };
State    state = IDLE;
uint32_t armedAtMs  = 0;
uint32_t runStartMs = 0;
uint8_t  runId      = 0;
bool     monitorOn  = false;
uint32_t lastTickMs = 0;
uint32_t tickCount  = 0;

// Controle
float    prevErr     = 0;
float    lastSeenErr = 0;
uint32_t lastSeenMs  = 0;

// ============================= ENCODERS =============================
//
// A interrupção só conta pulsos e guarda o instante do último.
// A cada amostra o rpm é calculado pelo método M/T:
//   rpm = 60e6 * pulsos / (PPR * tempo_entre_a_ultima_borda_da_janela_anterior_e_a_desta_em_us)
// Isso dá boa resolução mesmo com poucos pulsos por janela (disco de 20 furos).
// Janela sem pulso: o rpm vai caindo pelo limite 1 pulso / tempo decorrido,
// e zera depois de ENC_TIMEOUT_US.

struct Encoder {
  volatile uint32_t count    = 0;  // pulsos válidos (atualizado na interrupção)
  volatile uint32_t lastUs   = 0;  // instante do último pulso válido
  volatile uint32_t periodUs = 0;  // intervalo entre os dois últimos pulsos
  uint32_t prevCount  = 0;         // valores usados na amostra anterior
  uint32_t prevEdgeUs = 0;
  bool     stopped    = true;
  float    rpm        = 0;
};

Encoder encL, encR;

void IRAM_ATTR encoderEdge(Encoder &e) {
  uint32_t now = micros();
  uint32_t dt  = now - e.lastUs;
  if (dt < ENC_MIN_INTERVAL_US) return;
  e.periodUs = dt;
  e.lastUs   = now;
  e.count    = e.count + 1;
}

void IRAM_ATTR isrEncL() { encoderEdge(encL); }
void IRAM_ATTR isrEncR() { encoderEdge(encR); }

void updateRpm(Encoder &e, uint32_t nowUs) {
  noInterrupts();
  uint32_t count    = e.count;
  uint32_t lastUs   = e.lastUs;
  uint32_t periodUs = e.periodUs;
  interrupts();

  uint32_t dn = count - e.prevCount;

  if (dn > 0) {
    if (e.stopped) {
      // A roda estava parada: o tempo desde a borda antiga não serve.
      // Com 2 ou mais pulsos usa o período do último par; com 1 ainda não dá para saber.
      e.rpm = (dn >= 2 && periodUs > 0) ? 60e6f / (ENC_PPR * (float)periodUs) : 0.0f;
    } else {
      e.rpm = 60e6f * (float)dn / (ENC_PPR * (float)(lastUs - e.prevEdgeUs));
    }
    e.prevCount  = count;
    e.prevEdgeUs = lastUs;
    e.stopped    = false;
  } else {
    uint32_t since = nowUs - e.prevEdgeUs;
    if (e.stopped || since > ENC_TIMEOUT_US) {
      e.rpm     = 0;
      e.stopped = true;
    } else {
      float maxRpm = 60e6f / (ENC_PPR * (float)since);
      if (maxRpm < e.rpm) e.rpm = maxRpm;
    }
  }
}

uint16_t rpmX10(float rpm) {
  float v = rpm * 10.0f + 0.5f;
  if (v < 0.0f) v = 0.0f;
  if (v > 65535.0f) v = 65535.0f;
  return (uint16_t)v;
}

// ============================== MOTORES ==============================

int clampInt(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

// cmd de -255 a 255. Se sua ponte H for do tipo PWM + DIR (como o driver da RoboCore)
// ou tiver 2 entradas por motor (L9110S, MX1508), adapte só esta função.
void setMotor(const Motor &m, int cmd) {
  cmd = clampInt(cmd, -255, 255);
  if (m.invert) cmd = -cmd;
  if (cmd > 0) {
    analogWrite(m.in1, cmd);
    analogWrite(m.in2, cmd);
  } else if (cmd < 0) {
    analogWrite(m.in1, cmd);
    analogWrite(m.in2, cmd);
  } else {
    analogWrite(m.in1, LO_ANALOG);
    analogWrite(m.in2, LO_ANALOG);
  }
}

void initMotor(const Motor &m) {
  pinMode(m.in1, OUTPUT);
  pinMode(m.in2, OUTPUT);
  setMotor(m, 0);
}

// ========================== SENSOR DE LINHA ==========================

uint8_t readLineRaw() {
  uint8_t bits = 0;
  for (uint8_t i = 0; i < 8; i++) 
    bits |= (uint8_t)(digitalRead(LINE_PINS[i]) << i); // bitmasking
  return bits;
}

// Máscara com bit i = 1 quando o sensor i está sobre a linha
uint8_t onLineMask(uint8_t raw) {
  return (LINE_ON_LEVEL == HIGH) ? raw : (uint8_t)~raw;
}

// Controle PD sobre a posição da linha. Retorna false se a linha não foi vista.
bool followLine(uint8_t on, uint32_t nowMs, int &pwmL, int &pwmR) {
  int   n   = 0;
  float sum = 0;
  for (uint8_t i = 0; i < 8; i++) {
    if (on & (1u << i)) { sum += (float)i - 3.5f; n++; }
  }

  if (n > 0) {
    float err  = sum / n;   // de -3,5 (linha toda à esquerda) a +3,5 (toda à direita)
    float corr = KP * err + KD * (err - prevErr);
    prevErr     = err;
    lastSeenErr = err;
    lastSeenMs  = nowMs;
    // Linha à esquerda (err < 0): roda esquerda mais lenta, robô vira para a esquerda
    pwmL = clampInt((int)(BASE_PWM + corr), 0, 255);
    pwmR = clampInt((int)(BASE_PWM - corr), 0, 255);
    return true;
  }

  // Linha perdida: gira sobre uma roda para o lado onde ela foi vista por último
  if (lastSeenErr < 0) { pwmL = 0;          pwmR = SEARCH_PWM; }
  else                 { pwmL = SEARCH_PWM; pwmR = 0; }
  return false;
}

// ============================ MENSAGENS ============================

// Só escreve se houver um terminal aberto (o robô roda sem USB)
void msg(const char *s) {
  if (Serial) Serial.println(s);
}

void printStatus() {
  const char *st = (state == IDLE) ? "parado" : (state == ARMED ? "armado" : "gravando");
  char b[128];
  snprintf(b, sizeof(b), "# estado=%s amostras=%lu/%lu corridas=%u periodo=%lums",
           st, (unsigned long)nSamples, (unsigned long)MAX_SAMPLES,
           (unsigned)runId, (unsigned long)SAMPLE_PERIOD_MS);
  msg(b);
}

void printMonitor(uint8_t raw) {
  if (!Serial) return;
  char bits[9];
  for (uint8_t i = 0; i < 8; i++) bits[i] = (raw & (1u << i)) ? '1' : '0';
  bits[8] = '\0';
  uint16_t l = rpmX10(encL.rpm), r = rpmX10(encR.rpm);
  char b[80];
  snprintf(b, sizeof(b), "s0..s7=%s  rpm_l=%u.%u  rpm_r=%u.%u",
           bits, (unsigned)(l / 10), (unsigned)(l % 10), (unsigned)(r / 10), (unsigned)(r % 10));
  Serial.println(b);
}

void dumpCsv() {
  Serial.println("run,t_ms,s0,s1,s2,s3,s4,s5,s6,s7,rpm_l,rpm_r,pwm_l,pwm_r");
  char line[112];
  for (uint32_t k = 0; k < nSamples; k++) {
    const Sample &s = samples[k];
    int n = snprintf(line, sizeof(line), "%u,%lu", (unsigned)s.run, (unsigned long)s.t_ms);
    for (uint8_t i = 0; i < 8; i++) {
      n += snprintf(line + n, sizeof(line) - n, ",%u", (unsigned)((s.sensors >> i) & 1u));
    }
    snprintf(line + n, sizeof(line) - n, ",%u.%u,%u.%u,%d,%d",
             (unsigned)(s.rpm_l_x10 / 10), (unsigned)(s.rpm_l_x10 % 10),
             (unsigned)(s.rpm_r_x10 / 10), (unsigned)(s.rpm_r_x10 % 10),
             (int)s.pwm_l, (int)s.pwm_r);
    Serial.println(line);
  }
  Serial.println("#FIM");
}

// ========================== MÁQUINA DE ESTADOS ==========================

void arm() {
  if (state != IDLE) return;
  if (nSamples >= MAX_SAMPLES) {
    msg("# buffer cheio: descarregue (d) e apague (c)");
    return;
  }
  state     = ARMED;
  armedAtMs = millis();
  msg("# armado, iniciando em instantes");
}

void beginRun(uint32_t nowMs) {
  runId++;
  runStartMs  = nowMs;
  prevErr     = 0;
  lastSeenErr = 0;
  lastSeenMs  = nowMs;
  state       = RECORDING;
  char b[40];
  snprintf(b, sizeof(b), "# corrida %u gravando", (unsigned)runId);
  msg(b);
}

void stopRun(const char *why) {
  if (state == IDLE) return;
  state = IDLE;
  setMotor(MOT_L, 0);
  setMotor(MOT_R, 0);
  char b[96];
  snprintf(b, sizeof(b), "# parado (%s). %lu amostras no buffer", why, (unsigned long)nSamples);
  msg(b);
}

// ============================== ENTRADAS ==============================

bool buttonPressed() {
  static int      lastStable = HIGH, lastRead = HIGH;
  static uint32_t changedAt  = 0;
  int r = digitalRead(BUTTON_PIN);
  if (r != lastRead) { lastRead = r; changedAt = millis(); }
  if (millis() - changedAt > 30 && r != lastStable) {
    lastStable = r;
    return r == LOW;   // borda de descida = botão apertado
  }
  return false;
}

void handleSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    switch (c) {
      case 'r': arm(); break;
      case 's': stopRun("comando"); break;
      case 'd':
        if (state == IDLE) dumpCsv();
        else msg("# pare a gravacao antes (s)");
        break;
      case 'c':
        if (state == IDLE) { nSamples = 0; runId = 0; msg("# buffer apagado"); }
        break;
      case 'v':
        monitorOn = !monitorOn;
        msg(monitorOn ? "# monitor ligado" : "# monitor desligado");
        break;
      case '?': printStatus(); break;
      default: break;   // ignora \r, \n e o resto
    }
  }
}

// ============================ CICLO FIXO ============================

void tick() {
  uint32_t nowMs = millis();
  uint32_t nowUs = micros();

  updateRpm(encL, nowUs);
  updateRpm(encR, nowUs);
  uint8_t raw = readLineRaw();

  if (state == ARMED && nowMs - armedAtMs >= START_DELAY_MS) beginRun(nowMs);

  int pwmL = 0, pwmR = 0;
  if (state == RECORDING) {
    bool seen = followLine(onLineMask(raw), nowMs, pwmL, pwmR);

    Sample &s = samples[nSamples++];
    s.t_ms      = nowMs - runStartMs;
    s.rpm_l_x10 = rpmX10(encL.rpm);
    s.rpm_r_x10 = rpmX10(encR.rpm);
    s.pwm_l     = (int16_t)pwmL;
    s.pwm_r     = (int16_t)pwmR;
    s.sensors   = raw;
    s.run       = runId;

    if (!seen && nowMs - lastSeenMs > LOST_TIMEOUT_MS) {
      stopRun("linha perdida");
      pwmL = pwmR = 0;
    } else if (nSamples >= MAX_SAMPLES) {
      stopRun("buffer cheio");
      pwmL = pwmR = 0;
    }
  }

  setMotor(MOT_L, pwmL);
  setMotor(MOT_R, pwmR);

  // LED: aceso gravando, piscando enquanto armado
  bool led = (state == RECORDING) || (state == ARMED && ((nowMs / 100) % 2 == 0));
  digitalWrite(LED_BUILTIN, led ? HIGH : LOW);

  tickCount++;
  if (monitorOn && tickCount % (200 / SAMPLE_PERIOD_MS) == 0) printMonitor(raw);
}

// ============================ SETUP / LOOP ============================

void setup() {
  Serial.begin(115200);   // sem while(!Serial): o robô precisa rodar só na bateria

  for (uint8_t i = 0; i < 8; i++) pinMode(LINE_PINS[i], INPUT);

  pinMode(ENC_L_PIN, INPUT);
  pinMode(ENC_R_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_L_PIN), isrEncL, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_PIN), isrEncR, RISING);

  initMotor(MOT_L);
  initMotor(MOT_R);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  lastTickMs = millis();
}

void loop() {
  handleSerial();

  if (buttonPressed()) {
    if (state == IDLE) arm();
    else stopRun("botao");
  }

  uint32_t now = millis();
  if (now - lastTickMs >= SAMPLE_PERIOD_MS) {
    lastTickMs += SAMPLE_PERIOD_MS;
    // Se ficou para trás (por exemplo depois do dump), realinha em vez de rodar vários ticks seguidos
    if (now - lastTickMs >= SAMPLE_PERIOD_MS) lastTickMs = now;
    tick();
  }
}
