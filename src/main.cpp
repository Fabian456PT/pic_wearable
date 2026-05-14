#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"

MAX30105 particleSensor;

// O XIAO ESP32-C3 tem um LED no pino 10 (geralmente azul ou amarelo)
const int LED_PIN = 10; 

void setup() {
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);

    // Loop de espera para dares tempo ao Monitor Serial de abrir
    for(int i = 5; i > 0; i--) {
        digitalWrite(LED_PIN, LOW); // Liga LED (logica inversa em alguns XIAO)
        delay(500);
        digitalWrite(LED_PIN, HIGH); // Desliga LED
        delay(500);
        Serial.printf("A começar em %d...\n", i);
    }

    Serial.println("A inicializar I2C (Pinos 6 e 7)...");
    Wire.begin(6, 7); 

    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("ERRO: Sensor MAX3010X não encontrado!");
        Serial.println("Verifica se o SDA está no D4 e SCL no D5.");
        while (1) {
            // Pisca rápido se houver erro
            digitalWrite(LED_PIN, !digitalRead(LED_PIN));
            delay(100);
        }
    }

    Serial.println("Sensor detetado! A configurar...");
    particleSensor.setup(0x1F, 4, 2, 400, 411, 4096);
    Serial.println("Tudo pronto. Coloca o dedo no sensor.");
}

void loop() {
    long irValue = particleSensor.getIR();

    if (irValue < 50000) {
        Serial.println("Nenhum dedo detetado...");
    } else {
        Serial.print("IR Value: ");
        Serial.println(irValue);
    }
    
    delay(100); 
}