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
const byte RATE_SIZE = 10;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg = 0;

const float LIMITE_GIROSCOPIO = 4.5;
const float LIMITE_ACELERACAO = 15.0;
const int TEMPO_DEBOUNCE_BOT = 50;

int estadoBotao;
int ultimoEstadoBotao = HIGH;
unsigned long ultimoTempoClique = 0;
int eventoAtivo = 0;

unsigned long ultimoTempoEnvioBLE = 0;
const int INTERVALO_ENVIO_BLE = 1000;

// --- 5. CONTROLO DO MODO DE REPOUSO ---
const unsigned long TIMEOUT_SLEEP = 30000; // 30 segundos sem app
unsigned long tempoUltimaLigacao = 0;
bool sensoresSuspensos = false; // Apenas os sensores são suspensos, BLE continua sempre

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
  Wire.write(0x6B);  // PWR_MGMT_1
  Wire.write(0x40);  // Bit 6 = SLEEP
  Wire.endTransmission();

  // Limpa histórico cardíaco
  rateSpot = 0;
  beatAvg = 0;
  for (byte x = 0; x < RATE_SIZE; x++) rates[x] = 0;
}

void reativarSensores() {
  sensoresSuspensos = false;
  Serial.println(">>> SENSORES REATIVADOS <<<");

  // Acorda MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00); // Limpa bit de sleep
  Wire.endTransmission();
  delay(10);

  // Acorda MAX30102
  particleSensor.wakeUp();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);
}

void entrarEmComaProfundo() {
  Serial.println("\n>>> INICIAR ENCERRAMENTO TOTAL (DEEP SLEEP) <<<");
  particleSensor.setPulseAmplitudeRed(0);
  particleSensor.setPulseAmplitudeGreen(0);
  particleSensor.shutDown();
  
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);  
  Wire.write(0x40);  
  Wire.endTransmission();

  BLEDevice::deinit(true);

  esp_deep_sleep_enable_gpio_wakeup(1ULL << BOTAO_PIN, ESP_GPIO_WAKEUP_GPIO_LOW);
  Serial.println("A cortar energia... Boa noite!");
  Serial.flush();
  esp_deep_sleep_start(); 
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

  // >>> ADICIONA ESTE BLOCO NO INÍCIO DO LOOP <<<
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    comando.trim(); // Limpa espaços invisíveis ou quebras de linha
    
    if (comando == "sleep") {
      entrarEmComaProfundo(); // Chama a tua função se digitares "sleep"
    }
  }

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

  // Se sensores suspensos, não lê nada — ESP32 e BLE continuam normalmente
  if (sensoresSuspensos) {
    delay(100); // Pequena pausa para não spammar o CPU desnecessariamente
    return;
  }

  // ==========================================
  // 1. OUVIR O CORAÇÃO
  // ==========================================
  long irValue = particleSensor.getIR();
  if (checkForBeat(irValue) == true) {
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      if (rates[0] == 0) {
        for (byte x = 0; x < RATE_SIZE; x++) rates[x] = (byte)beatsPerMinute;
      } else {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;
      }

      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++) beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
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
  // 3. EMPACOTAR E ENVIAR DADOS (1x por Segundo)
  // ==========================================
  if (tempoAtual - ultimoTempoEnvioBLE >= INTERVALO_ENVIO_BLE) {
    ultimoTempoEnvioBLE = tempoAtual;

    long somaGSR = 0;
    for(int i = 0; i < 5; i++) somaGSR += analogRead(GSR_PIN);
    int edaFinal = somaGSR / 5;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    int flagMovimento = 0;
    float magnitude = sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2));

    if (abs(g.gyro.x) > LIMITE_GIROSCOPIO || abs(g.gyro.y) > LIMITE_GIROSCOPIO || abs(g.gyro.z) > LIMITE_GIROSCOPIO ||
        abs(magnitude - 9.8) > LIMITE_ACELERACAO) {
      flagMovimento = 1;
      edaFinal = 0;
      beatAvg = 0;
      rateSpot = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++) rates[x] = 0;
      Serial.println("RUÍDO DETETADO! Dados limpos para envio.");
    }

    if (deviceConnected) {
      char bufferJSON[100];
      snprintf(bufferJSON, sizeof(bufferJSON), "{\"bpm\":%d,\"eda\":%d,\"mov\":%d,\"evt\":%d}",
               beatAvg, edaFinal, flagMovimento, eventoAtivo);
      pCharacteristic->setValue(bufferJSON);
      pCharacteristic->notify();
      Serial.print("BLE Enviado: ");
      Serial.println(bufferJSON);
    } else {
      Serial.print("Local -> BPM: "); Serial.print(beatAvg);
      Serial.print(" | EDA: "); Serial.print(edaFinal);
      Serial.print(" | Mov: "); Serial.print(flagMovimento);
      Serial.print(" | Botao (evt): "); Serial.println(eventoAtivo);
    }

    eventoAtivo = 0;
  }
}