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
const int GSR_PIN = A0;   // Sensor de Suor (Grove)
const int BOTAO_PIN = D3; // Botão Marcador de Evento

// --- 2. DEFINIÇÕES BLUETOOTH (BLE) ---
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;

// --- 3. OBJETOS DOS SENSORES ---
MAX30105 particleSensor;
Adafruit_MPU6050 mpu;

// --- 4. VARIÁVEIS GLOBAIS ---
// Lógica do Coração
const byte RATE_SIZE = 10;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg = 0;
// Limites de Calibração (Filtro de Ruído)
const float LIMITE_GIROSCOPIO = 4.5;    // rad/s (Rotação para rejeitar dados)
const float LIMITE_ACELERACAO = 15.0;   // m/s^2 (Impacto para rejeitar dados)
const int TEMPO_DEBOUNCE_BOT = 50;      // ms (Filtro anti-vibração do botão)

// Lógica do Botão
int estadoBotao;
int ultimoEstadoBotao = HIGH;
unsigned long ultimoTempoClique = 0;
int eventoAtivo = 0; // Fica a 1 quando clicas, volta a 0 após o envio BLE

// Cronómetro Mestre (Substitui o delay)
unsigned long ultimoTempoEnvioBLE = 0;
const int INTERVALO_ENVIO_BLE = 1000; // 1000 milissegundos = 1 segundo

// Callbacks para detetar quando a App Python se liga/desliga
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; };
    void onDisconnect(BLEServer* pServer) { 
      deviceConnected = false; 
      pServer->startAdvertising(); // Volta a procurar a App
    }
};

void setup() {
  Serial.begin(115200);
  
  // AVISO: Quando ligares à bateria sem PC, apaga ou comenta a linha abaixo!
  // while(!Serial); 

  Serial.println("A iniciar o Wearable de Saúde Mental...");

  // Iniciar Botão com resistência interna
  pinMode(BOTAO_PIN, INPUT_PULLUP);

  // Iniciar linha de comunicação I2C
  Wire.begin();

  // Iniciar MAX30102 (Coração)
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("Erro: MAX30102 não encontrado.");
    while (1);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);

  // Iniciar MPU6050 (Movimento)
  if (!mpu.begin()) {
    Serial.println("Erro: MPU6050 não encontrado.");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Iniciar GSR (Resolução de leitura a 12-bits)
  analogReadResolution(12);

  // Iniciar Bluetooth
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

  Serial.println("Sistema Pronto! A aguardar ligação da App...");
}

void loop() {
  // Tempo atual em milissegundos
  unsigned long tempoAtual = millis();

  // ==========================================
  // 1. OUVIR O CORAÇÃO (Corre à velocidade máxima)
  // ==========================================
  long irValue = particleSensor.getIR();
  if (checkForBeat(irValue) == true) {
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);
    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++) beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  // ==========================================
  // 2. LER O BOTÃO (Com filtro Anti-Vibração)
  // ==========================================
  int leituraBotao = digitalRead(BOTAO_PIN);
  if (leituraBotao != ultimoEstadoBotao) {
    ultimoTempoClique = tempoAtual;
  }
  if ((tempoAtual - ultimoTempoClique) > TEMPO_DEBOUNCE_BOT) {
    if (leituraBotao != estadoBotao) {
      estadoBotao = leituraBotao;
      // Se carregou, guarda na memória que o evento aconteceu!
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
    ultimoTempoEnvioBLE = tempoAtual; // Reinicia o cronómetro de envio

    // A. Ler o Suor (GSR)
    // Fazemos 5 leituras rápidas e calculamos a média para estabilidade
    long somaGSR = 0;
    for(int i = 0; i < 5; i++) {
      somaGSR += analogRead(GSR_PIN);
    }
    int edaFinal = somaGSR / 5;

    // B. Ler Giroscópio (O Detetor de Mentiras do Corpo)
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    
    int flagMovimento = 0;
    
    // Calcula a força total independentemente da orientação do relógio (Teorema de Pitágoras em 3D)
    float magnitude = sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.y, 2) + pow(a.acceleration.z, 2));
    
    // Se o braço sofrer rotação rápida (> 4.5 rad/s) OU impacto brusco em qualquer direção (Magnitude - Gravidade > 15 m/s^2)
    if (abs(g.gyro.x) > LIMITE_GIROSCOPIO || abs(g.gyro.y) > LIMITE_GIROSCOPIO || abs(g.gyro.z) > LIMITE_GIROSCOPIO || 
        abs(magnitude - 9.8) > LIMITE_ACELERACAO) {
      
      flagMovimento = 1; 
      
      // >>> FILTRO DE HARDWARE ATIVADO <<<
      // Se há movimento excessivo, eliminamos os dados ruidosos
      edaFinal = 0;
      beatAvg = 0;
      
      // Limpamos o histórico do array cardíaco para não arrastar o erro 
      // para os segundos seguintes quando o movimento parar
      rateSpot = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++) rates[x] = 0;
      
      Serial.println("RUÍDO DETETADO! Dados limpos para envio.");
    }

    // C. O Teu Pacote de Dados JSON
    // Só envia por Bluetooth se a App estiver conectada
    if (deviceConnected) {
      // Criamos uma caixa de memória fixa de 100 caracteres (não fragmenta a memória)
      char bufferJSON[100]; 
      
      // Injetamos os números diretamente no texto formatado
      snprintf(bufferJSON, sizeof(bufferJSON), "{\"bpm\":%d,\"eda\":%d,\"mov\":%d,\"evt\":%d}", 
               beatAvg, edaFinal, flagMovimento, eventoAtivo);

      pCharacteristic->setValue(bufferJSON);
      pCharacteristic->notify();

      Serial.print("BLE Enviado: ");
      Serial.println(bufferJSON);

      
    } else {
      // Se não houver App ligada, imprime no ecrã para tu testares
      Serial.print("Local -> BPM: "); Serial.print(beatAvg);
      Serial.print(" | EDA: "); Serial.print(edaFinal);
      Serial.print(" | Mov: "); Serial.println(flagMovimento);
      Serial.print(" | Botao (evt): "); Serial.println(eventoAtivo);
    }
    // Limpa a memória do clique do botão depois da App o receber
      eventoAtivo = 0; 
  }
}