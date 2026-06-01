#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- 1. DEFINIÇÕES DE PINOS ---
const int GSR_PIN = A0;
const int BOTAO_PIN = D3;

// --- 2. DEFINIÇÕES BLUETOOTH (BLE) ---
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool acabouDeConectar = false;
bool acabouDeDesconectar = false;

// --- 3. OBJETOS DOS SENSORES ---
MAX30105 particleSensor;
Adafruit_MPU6050 mpu;

// --- 4. VARIÁVEIS GLOBAIS ---

// Lógica do Coração — Média dos 3 segundos
const byte RATE_SIZE = 10;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;

// Acumulador para média dos 3 segundos
int bpmAcumulado = 0;
int bpmContagem = 0;
float beatAvgEnvio = 0.0;     // Último valor médio válido para enviar (float para aplicar EMA)
int ultimoBpmValido = 0;  // Guarda sempre o último BPM não-zero para não enviar 0

const float LIMITE_GIROSCOPIO = 4.5;
const float LIMITE_ACELERACAO = 15.0;
const int TEMPO_DEBOUNCE_BOT = 50;

int estadoBotao;
int ultimoEstadoBotao = HIGH;
unsigned long ultimoTempoClique = 0;
int eventoAtivo = 0;

unsigned long ultimoTempoEnvioBLE = 0;
const int INTERVALO_ENVIO_BLE = 3000; // 3 segundos

// --- 5. CONTROLO DO MODO DE REPOUSO ---
const unsigned long TIMEOUT_SLEEP = 30000;
unsigned long tempoUltimaLigacao = 0;
bool sensoresSuspensos = false;

// Callbacks BLE — só mudam flags
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      acabouDeConectar = true;
    }
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      acabouDeDesconectar = true;
      pServer->startAdvertising();
    }
};

void suspenderSensores() {
  sensoresSuspensos = true;
  Serial.println(">>> SENSORES SUSPENSOS (BLE continua ativo) <<<");
  Serial.flush();
  
  // Desliga LEDs do MAX30102 e coloca em shutdown
  particleSensor.setPulseAmplitudeRed(0);
  particleSensor.setPulseAmplitudeGreen(0);
  particleSensor.shutDown();

  // Coloca MPU6050 em sleep via registo I2C
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x40);
  Wire.endTransmission();

  // Limpa acumuladores mas preserva o último valor válido
  rateSpot = 0;
  bpmAcumulado = 0;
  bpmContagem = 0;
  beatAvgEnvio = 0;
  for (byte x = 0; x < RATE_SIZE; x++) rates[x] = 0;
}

void reativarSensores() {
  sensoresSuspensos = false;
  Serial.println(">>> SENSORES REATIVADOS <<<");

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(10);

  particleSensor.wakeUp();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);
}

void setup() {
  Serial.begin(115200);
  Serial.println("A iniciar o Wearable de Saúde Mental...");

  pinMode(BOTAO_PIN, INPUT_PULLUP);
  Wire.begin();

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("Erro: MAX30102 não encontrado.");
    while (1);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);

  if (!mpu.begin()) {
    Serial.println("Erro: MPU6050 não encontrado.");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  analogReadResolution(12);

  BLEDevice::init("Mindsight");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();

  tempoUltimaLigacao = millis();
  Serial.println("Sistema Pronto! A aguardar ligação da App...");
}

void loop() {
  unsigned long tempoAtual = millis();

  // ==========================================
  // GESTÃO DE EVENTOS BLE
  // ==========================================
  if (acabouDeConectar) {
    acabouDeConectar = false;
    if (sensoresSuspensos) reativarSensores();
    Serial.println("App ligada!");
  }

  if (acabouDeDesconectar) {
    acabouDeDesconectar = false;
    tempoUltimaLigacao = tempoAtual;
    Serial.println("App desligada. Sensores suspendem em 30s...");
  }

  // ==========================================
  // TIMEOUT: Suspender sensores se não houver app
  // ==========================================
  if (!deviceConnected && !sensoresSuspensos) {
    if (tempoAtual - tempoUltimaLigacao >= TIMEOUT_SLEEP) {
      suspenderSensores();
    }
  }

  if (sensoresSuspensos) {
    delay(100);
    return;
  }

  // ==========================================
  // 1. OUVIR O CORAÇÃO (corre à velocidade máxima)
  //    Acumula batimentos — a média é calculada no envio
  // ==========================================
  long irValue = particleSensor.getIR();
  if (checkForBeat(irValue) == true) {
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute > 20 && beatsPerMinute < 255) {
      // Acumula para a média dos 3 segundos
      bpmAcumulado += (int)beatsPerMinute;
      bpmContagem++;
    }
  }

  // ==========================================
  // 2. LER O BOTÃO
  // ==========================================
  int leituraBotao = digitalRead(BOTAO_PIN);
  if (leituraBotao != ultimoEstadoBotao) {
    ultimoTempoClique = tempoAtual;
  }
  if ((tempoAtual - ultimoTempoClique) > TEMPO_DEBOUNCE_BOT) {
    if (leituraBotao != estadoBotao) {
      estadoBotao = leituraBotao;
      if (estadoBotao == LOW) {
        eventoAtivo = 1;
        Serial.println(">>> BOTAO FISICO CLICADO! <<<");
      }
    }
  }
  ultimoEstadoBotao = leituraBotao;

  // ==========================================
  // 3. EMPACOTAR E ENVIAR DADOS (1x a cada 3 segundos)
  // ==========================================
  if (tempoAtual - ultimoTempoEnvioBLE >= INTERVALO_ENVIO_BLE) {
    ultimoTempoEnvioBLE = tempoAtual;

    // A. Calcular média do BPM dos últimos 3 segundos COM FILTRO EMA
    if (bpmContagem > 0) {
      float mediaJanelaAtual = (float)bpmAcumulado / bpmContagem;
      
      if (beatAvgEnvio == 0.0) {
        beatAvgEnvio = mediaJanelaAtual; // Se for a primeira vez, assume o valor
      } else {
        // Aplica o filtro de estabilização (suaviza os saltos)
        float alpha = 0.3; 
        beatAvgEnvio = (mediaJanelaAtual * alpha) + (beatAvgEnvio * (1.0 - alpha));
      }
      ultimoBpmValido = (int)beatAvgEnvio; // Guarda como último valor conhecido
    } 
    
    // Converte a média calculada para inteiro para enviar
    int bpmFinalParaEnviar = (beatAvgEnvio > 0) ? (int)beatAvgEnvio : ultimoBpmValido;

    // Reinicia acumuladores para os próximos 3 segundos
    bpmAcumulado = 0;
    bpmContagem = 0;

    // B. Ler GSR
    long somaGSR = 0;
    for(int i = 0; i < 5; i++) somaGSR += analogRead(GSR_PIN);
    
    float mediaRaw12 = somaGSR / 5.0; // 1. Média da leitura bruta a 12-bits (0 a 4095)
    float raw10 = mediaRaw12 / 4.0; // 2. Converter para a escala de 10-bits (0 a 1023) exigida pela fórmula oficial
    float edaFinal = 0.0; // 3. Aplicar a fórmula de conversão para microSiemens
    
    // Proteção matemática: 512 é o valor de circuito aberto (sem dedo)
    // Se o valor passar de 512, a matemática daria números negativos
    if (raw10 < 512.0) { 
      float resistenciaOhms = ((1024.0 + 2.0 * raw10) * 10000.0) / (512.0 - raw10);
      edaFinal = 1000000.0 / resistenciaOhms; // Condutância em microSiemens (uS)
    }

    // C. Ler MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    int flagMovimento = 0;
    float magnitude = sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2));

    if (abs(g.gyro.x) > LIMITE_GIROSCOPIO || abs(g.gyro.y) > LIMITE_GIROSCOPIO || abs(g.gyro.z) > LIMITE_GIROSCOPIO ||
        abs(magnitude - 9.8) > LIMITE_ACELERACAO) {
      flagMovimento = 1;
      edaFinal = 0;
      // BPM: NÃO zeramos — mantemos o último valor válido
      // Apenas limpamos o acumulador para não misturar dados ruidosos
      bpmAcumulado = 0;
      bpmContagem = 0;
      Serial.println("RUÍDO DETETADO! EDA limpa, BPM mantém último valor válido.");
    }

    // D. Enviar JSON
    if (deviceConnected) {
      char bufferJSON[100];
      snprintf(bufferJSON, sizeof(bufferJSON), "{\"bpm\":%d,\"eda\":%.2f,\"mov\":%d,\"evt\":%d}",
               bpmFinalParaEnviar, edaFinal, flagMovimento, eventoAtivo);
      pCharacteristic->setValue(bufferJSON);
      pCharacteristic->notify();
      Serial.print("BLE Enviado: ");
      Serial.println(bufferJSON);
    } else {
      Serial.print("Local -> BPM: "); Serial.print(bpmFinalParaEnviar);
      Serial.print(" | EDA: "); Serial.print(edaFinal);
      Serial.print(" | Mov: "); Serial.print(flagMovimento);
      Serial.print(" | Botao (evt): "); Serial.println(eventoAtivo);
    }

    eventoAtivo = 0;
  }
}